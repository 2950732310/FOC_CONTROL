#include "foc_thread.h"
#include "dwt.h"
#include "adc.h"
#include "serial_thread.h"
#include "pid.h"

static PIController_t g_id_pi = {0};
static PIController_t g_iq_pi = {0};
static PIController_t g_vel_pi = {0};
static uint8_t g_torque_loop_ready = 0;

static void FOC_UpdateCurrentSampling(void)
{
  adc_buf[0] = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
  adc_buf[1] = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);

  motor.foc.i_a = ((float)adc_buf[0] - motor.foc.flash_data.ia_zero) * SCALE;
  motor.foc.i_b = ((float)adc_buf[1] - motor.foc.flash_data.ib_zero) * SCALE;
  motor.foc.i_c = -(motor.foc.i_a + motor.foc.i_b);
}

/* 电机结构体初始化 */
MOTOR_DATA motor = {
    .State_Mode = STATE_MODE_RUNNING,
    .Control_Mode = CONTROL_MODE_VELOCITY,
    .foc ={ 
            .flash_data = {
              .ia_zero = 0.0f,
              .ib_zero = 0.0f,
              .id_kp = 1.0f,
              .id_ki = 0.1f,
              .iq_kp = 1.1f,
              .iq_ki = 0.3f,
              .vel_kp = 0.5f,
              .vel_ki = 0.05f,
            },
            .vbus = BATVEL,
            .inv_vbus = INVBATVEL,
            .vq_set = 5.0f,
            .vd_set = 0.0f,
            .i_d_ref = 0.0f,    
            .i_q_ref = 2.5f,    
            .vel_ref = 5.0f,
            .vel_fb = 0.0f,
            .flash_data.ia_zero = 0.0f,
            .flash_data.ib_zero = 0.0f,
        },
    .mt6816 = &encoder_data,
};



void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  switch (motor.State_Mode)
  {
  /* 空闲模式：电机停转且失能 */
  case STATE_MODE_IDLE:
    motor.foc.v_d = 0.0f;
    motor.foc.v_q = 0.0f;
    Foc_Set_Pwm(0.0f, 0.0f, 0.0f);
    break;
  /* 检测模式：电流校准设备检测等功能 */
  case STATE_MODE_DETECTING:
    if(command_flag.current_zero == 1)
    {
      osThreadFlagsSet(focControlThreaHandle, 0x01);
    }
    break;

  /* 运行模式：电机正常运行 */
  case STATE_MODE_RUNNING:
    FOC_UpdateCurrentSampling();           // 更新电流采样值
    GetMotor_Angle(motor.mt6816); // 获取机械角度
    switch (motor.Control_Mode)
    {
    case CONTROL_MODE_OPEN: // 开环控制
      Open_Loop_Control(&motor);
      break;
    case CONTROL_MODE_TORQUE: // 力矩控制模式
      Torque_Control(&motor);
      break;
    case CONTROL_MODE_VELOCITY: // 速度闭环控制模式
      Velocity_Control(&motor);
      break;
    case CONTROL_MODE_POSITION: // 位置闭环控制模式
      break;
    }
    break;

  /* 守护模式 */
  case STATE_MODE_GUARD:
    break;

  /* 默认状态 */
  default: // 默认状态
    break;
  }

}


void FOC_Control(void *argument)
{
  while (1)
  {
    if (motor.State_Mode == STATE_MODE_DETECTING)
    {
      if (command_flag.current_zero != 0)
      {
        osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
        command_flag.current_zero = 0;
        FOC_CurrentOffsetCalibration(&motor);
        osSemaphoreRelease(flashSemHandle); 
      }
    }else if (command_flag.flash_control != 0)
    {
      command_flag.flash_control = 0;
      osSemaphoreRelease(flashSemHandle); 
    }
    osDelay(10);
  }
}

void FOC_Align(MOTOR_DATA *motor)
{
  const uint8_t sample_count = 10;
  uint32_t sum_count = 0;

  motor->foc.v_d = 0.0f;
  motor->foc.v_q = 0.0f;
  motor->mt6816->elec_angle = 0.0f;

  Sin_Cos_Value(&motor->foc, motor->mt6816->elec_angle);

  // 渐进式增加 d 轴电压，避免对齐时机械震动过大
  for (float v = 0.0f; v <= 2.5f; v += 0.01f)
  {
    motor->foc.v_d = v;
    Inv_Park(&motor->foc);
    motor->foc.Ualpha_norm = motor->foc.v_alpha * INVBATVEL;
    motor->foc.Ubeta_norm = motor->foc.v_beta * INVBATVEL;
    Svpwm(&motor->foc);
    Foc_Set_Pwm(motor->foc.dtc_a, motor->foc.dtc_b, motor->foc.dtc_c);
    DWT_Delay_ms(2);
  }

  // 等待转子稳定
  DWT_Delay_ms(1000);

  // 采样多次并取平均，得到稳定的编码器零点计数
  // mt6816 驱动中偏移量用于 count_in_cpr_ - encoder_offset，
  // 所以这里保存的是 CPR 计数值，而不是弧度值
  for (uint8_t i = 0; i < sample_count; i++)
  {
    GetMotor_Angle(motor->mt6816);
    sum_count += (uint32_t)motor->mt6816->count_in_cpr_;
    DWT_Delay_ms(10);
  }

  motor->mt6816->encoder_offset = (float)sum_count / (float)sample_count;

  // 对齐结束，清零输出电压
  motor->foc.v_d = 0.0f;
  motor->foc.v_q = 0.0f;
  Foc_Set_Pwm(0.0f, 0.0f, 0.0f);
}

MOTOR_ERROR Open_Loop_Control(MOTOR_DATA *motor)
{
  /* 获取当前电角度 */
  float elec_angle = Normalize_Angle(motor->mt6816->elec_angle);

  /* 设置电压 */
  motor->foc.v_d = 0.0f;
  motor->foc.v_q = Limit(motor->foc.vq_set, -MAX_V_LIMIT, MAX_V_LIMIT);

  /* 坐标变换 */
  Sin_Cos_Value(&motor->foc, elec_angle);
  Inv_Park(&motor->foc);

  /* svpwm 输出 */
  motor->foc.Ualpha_norm = motor->foc.v_alpha * INVBATVEL;
  motor->foc.Ubeta_norm = motor->foc.v_beta * INVBATVEL;
  if (0 == Svpwm(&motor->foc))
  {
    Foc_Set_Pwm(motor->foc.dtc_a, motor->foc.dtc_b, motor->foc.dtc_c);
    return M_OK;
  }

  return M_ERR;
}


MOTOR_ERROR Torque_Control(MOTOR_DATA *motor)
{
  if (motor == NULL)
  {
    return M_ERR;
  }

  if (g_torque_loop_ready == 0)
  {
    PIController_Init(&g_id_pi, motor->foc.flash_data.id_kp, motor->foc.flash_data.id_ki, -MAX_V_LIMIT, MAX_V_LIMIT);
    PIController_Init(&g_iq_pi, motor->foc.flash_data.iq_kp, motor->foc.flash_data.iq_ki, -MAX_V_LIMIT, MAX_V_LIMIT);
    g_torque_loop_ready = 1;
  }

  Clarke(&motor->foc);
  Sin_Cos_Value(&motor->foc, Normalize_Angle(motor->mt6816->elec_angle));
  Park(&motor->foc);

  const float id_ref = Limit(motor->foc.i_d_ref, -MAX_V_LIMIT, MAX_V_LIMIT);
  const float iq_ref = Limit(motor->foc.i_q_ref, -MAX_V_LIMIT, MAX_V_LIMIT);
  const float error_d = id_ref - motor->foc.i_d;
  const float error_q = iq_ref - motor->foc.i_q;

  motor->foc.v_d = PIController_Update(&g_id_pi, error_d, CURRENT_MEASURE_PERIOD);
  motor->foc.v_q = PIController_Update(&g_iq_pi, error_q, CURRENT_MEASURE_PERIOD);

  motor->foc.v_d = Limit(motor->foc.v_d, -MAX_V_LIMIT, MAX_V_LIMIT);
  motor->foc.v_q = Limit(motor->foc.v_q, -MAX_V_LIMIT, MAX_V_LIMIT);

  Inv_Park(&motor->foc);
  motor->foc.Ualpha_norm = motor->foc.v_alpha * INVBATVEL;
  motor->foc.Ubeta_norm = motor->foc.v_beta * INVBATVEL;

  if (0 == Svpwm(&motor->foc))
  {
    Foc_Set_Pwm(motor->foc.dtc_a, motor->foc.dtc_b, motor->foc.dtc_c);
    return M_OK;
  }

  return M_ERR;
}


MOTOR_ERROR Velocity_Control(MOTOR_DATA *motor)
{
  if (motor == NULL || motor->mt6816 == NULL)
  {
    return M_ERR;
  }

  // if (g_vel_loop_ready == 0)
  // {
    PIController_Init(&g_vel_pi, motor->foc.flash_data.vel_kp, motor->foc.flash_data.vel_ki, -MAX_V_LIMIT, MAX_V_LIMIT);
  //   g_vel_loop_ready = 1;
  // }

  /* 1. 更新速度反馈 */
  motor->foc.vel_fb = motor->mt6816->vel_estimate_;

  /* 2. 速度PI控制器输出电流参考值 i_q_ref */
  const float vel_ref = Limit(motor->foc.vel_ref, -50.0f, 50.0f);
  const float vel_error = vel_ref - motor->foc.vel_fb;
  motor->foc.i_q_ref = PIController_Update(&g_vel_pi, vel_error, CURRENT_MEASURE_PERIOD);
  motor->foc.i_q_ref = Limit(motor->foc.i_q_ref, -MAX_V_LIMIT, MAX_V_LIMIT);

  /* 3. d轴电流参考设为0（不控制id），让力矩完全由iq承担 */
  motor->foc.i_d_ref = 0.0f;

  /* 4. 调用力矩控制实现电流闭环 */
  return Torque_Control(motor);
  
  // motor->foc.v_d = 0.0f;
  // motor->foc.vq_set = PIController_Update(&g_vel_pi, vel_error, CURRENT_MEASURE_PERIOD);
  // motor->foc.vq_set = Limit(motor->foc.vq_set, -MAX_V_LIMIT, MAX_V_LIMIT);
  // return Open_Loop_Control(motor);
}

/**
 * @brief 电流零点校准
 * @param motor: MOTOR_DATA 结构体指针
 * @note  关闭PWM驱动使电流为零，利用ADC连续DMA采样取平均值
 *        得到A/B相电流零点偏移，结果存入 motor.foc.i_a/b_offset
 *
 * 使用示例:
 *   FOC_CurrentOffsetCalibration(&motor);
 *   // 之后在电流采样处减去偏移:
 *   raw_a = adc_buf[0] - motor.foc.i_a_offset;
 *   raw_b = adc_buf[1] - motor.foc.i_b_offset;
 */
void FOC_CurrentOffsetCalibration(MOTOR_DATA *motor)
{
  uint32_t sum_a = 0;
  uint32_t sum_b = 0;
  const uint16_t samples = 256;
  uint16_t i;
  uint16_t raw_a = 0;
  uint16_t raw_b = 0;

  /* 1. 先停止 PWM 输出并关闭驱动，确保电流回路处于零激励状态 */
  motor->foc.v_d = 0.0f;
  motor->foc.v_q = 0.0f;
  Foc_Set_Pwm(0.0f, 0.0f, 0.0f);
  disable_foc_pwm_driver();
  Foc_Pwm_LowSides();

  /* 2. 等待 ADC/电流回路稳定 */
  DWT_Delay_ms(20);

  /* 3. 当前配置是注入通道 + TIM1触发 + 中断回调，不使用 DMA。
   *    因此直接读取注入通道数据寄存器，并多次取平均。 */
  for (i = 0; i < samples; i++)
  {
    DWT_Delay_us(50);
    raw_a = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    raw_b = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
    sum_a += raw_a;
    sum_b += raw_b;
  }

  /* 4. 保存零点偏移 */
  motor->foc.flash_data.ia_zero = (float)sum_a / (float)samples;
  motor->foc.flash_data.ib_zero = (float)sum_b / (float)samples;

  /* 5. 恢复输出，避免后续控制被卡住 */
  motor->foc.v_d = 0.0f;
  motor->foc.v_q = 0.0f;
  Foc_Set_Pwm(0.0f, 0.0f, 0.0f);
  DWT_Delay_ms(1);
  enable_foc_pwm_driver();
}

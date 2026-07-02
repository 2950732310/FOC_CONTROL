#include "foc_thread.h"
#include "dwt.h"
#include "adc.h"

static void FOC_UpdateCurrentSampling(void)
{
  adc_buf[0] = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
  adc_buf[1] = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);

  motor.foc.i_a = (float)adc_buf[0] - motor.foc.flash_data.ia_zero;
  motor.foc.i_b = (float)adc_buf[1] - motor.foc.flash_data.ib_zero;
  motor.foc.i_c = -(motor.foc.i_a + motor.foc.i_b);
}

/* 电机结构体初始化 */
MOTOR_DATA motor = {
    .State_Mode = STATE_MODE_RUNNING,
    .Control_Mode = CONTROL_MODE_OPEN,
    .foc =
        {
            .vbus = BATVEL,
            .inv_vbus = INVBATVEL,
            .vq_set = 5.0f,
            .vd_set = 0.0f,
            .flash_data.ia_zero = 0.0f,
            .flash_data.ib_zero = 0.0f,
        },
    .mt6816 = &encoder_data,
};

// /* 通知FOC线程采样完成 */
// osThreadFlagsSet(focControlThreaHandle, 0x01);
// /* 阻塞等待采样完成通知 */
//   osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);

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
    case CONTROL_MODE_TORQUE: // 电压力矩控制模式
      break;
    case CONTROL_MODE_VELOCITY: // 速度闭环控制模式
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

/* FOC 线程函数 ADC采样完成自动调用 */
void FOC_Control(void *argument)
{
  while (1)
  {
    // GetMotor_Angle(motor.mt6816); // 获取机械角度
     osDelay(100);
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
  uint32_t sum_a = 0, sum_b = 0;
  const uint16_t samples = 256;
  uint16_t i;

  /* 1. 关闭PWM驱动，使电机相电流为零 */
  disable_foc_pwm_driver();
  Foc_Pwm_LowSides(); // 三相下管导通，确保电流回路

  /* 2. 等待电流稳定 */
  DWT_Delay_ms(10);

  /* 3. 多次采样取平均（ADC由TIM1触发+DMA连续搬运，adc_buf持续更新） */
  for (i = 0; i < samples; i++)
  {
    sum_a += adc_buf[0];
    sum_b += adc_buf[1];
    DWT_Delay_us(50); // 等待下一个PWM周期的新数据
  }

  /* 4. 计算偏移均值 */
  motor->foc.flash_data.ia_zero = (float)sum_a / (float)samples;
  motor->foc.flash_data.ib_zero = (float)sum_b / (float)samples;

  /* 5. 恢复PWM驱动 */
  motor->foc.v_d = 0.0f;
  motor->foc.v_q = 0.0f;
  Foc_Set_Pwm(0.0f, 0.0f, 0.0f);
  DWT_Delay_ms(1);
  enable_foc_pwm_driver();
}

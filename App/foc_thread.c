#include "foc_thread.h"
#include "dwt.h"

/* 电机结构体初始化 */
MOTOR_DATA motor = {
    .State_Mode = STATE_MODE_RUNNING,
    .Control_Mode = CONTROL_MODE_OPEN,
    .foc =
        {
            .vbus = BATVEL,
            .inv_vbus = INVBATVEL,
            .vq_set = 3.0f,
            .vd_set = 0.0f,
            .flash_data.ia_zero = 0.0f,
            .flash_data.ib_zero = 0.0f,
        },
    .mt6816 =
        {
            .angle = 0.0f,
            .radian = 0.0f,
            .dir = 0,
        },
};




/* FOC 线程函数 ADC采样完成自动调用 */
void FOC_Control(void *argument)
{
  while (1)
  {
    /* 阻塞等待采样完成通知 */
    osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);

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
      GetMotor_Angle(&motor.mt6816); // 获取机械角度
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
}

void FOC_Align(MOTOR_DATA *motor)
{

  motor->foc.v_q = 0.0f;
  motor->foc.theta = 0;

  Sin_Cos_Value(&motor->foc);

  // 渐进式增加电压，减少震动
  for (float v = 0; v <= 2.5f; v += 0.01f)
  {
    motor->foc.v_d = v;
    Inv_Park(&motor->foc);
    motor->foc.Ualpha_norm = motor->foc.v_alpha * INVBATVEL;
    motor->foc.Ubeta_norm = motor->foc.v_beta * INVBATVEL;
    Svpwm(&motor->foc);
    Foc_Set_Pwm(motor->foc.dtc_a, motor->foc.dtc_b, motor->foc.dtc_c);
    DWT_Delay_ms(2);
  }

  // 等待转子彻底稳定
  DWT_Delay_ms(1000);

  // 多次采样取平均值，过滤 mt6816 的噪声
  float sum_angle = 0;
  for (int i = 0; i < 10; i++)
  {
    GetMotor_Angle(&motor->mt6816);
    sum_angle += motor->mt6816.radian;
    DWT_Delay_ms(10);
  }
  float avg_mech_angle = sum_angle / 10.0f * POLE_PAIRS;
  motor->foc.theta_offset = Normalize_Angle(avg_mech_angle);

  // 对齐结束，电压清零
  motor->foc.v_d = 0;
  Foc_Set_Pwm(motor->foc.dtc_a, motor->foc.dtc_b, motor->foc.dtc_c);
}

MOTOR_ERROR Open_Loop_Control(MOTOR_DATA *motor)
{
  /* 获取电角度 */
  motor->foc.theta = Normalize_Angle(motor->mt6816.radian * POLE_PAIRS -
                                     motor->foc.theta_offset);

  /* 设置电压 */
  motor->foc.v_d = 0.0f;
  motor->foc.v_q = Limit(motor->foc.vq_set, -MAX_V_LIMIT, MAX_V_LIMIT);

  /* 坐标变换 */
  Sin_Cos_Value(&motor->foc);
  Inv_Park(&motor->foc);

  /* svpwm输出 */
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

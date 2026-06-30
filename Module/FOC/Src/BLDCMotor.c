#include "BLDCMotor.h"

/**
 * @brief 启动FOC电机PWM输出
 * @param None
 * @retval None
 */
void Foc_Pwm_Start(void)
{
    // 设置初始占空比为 50% (让半桥输出 VCC/2，电机不转)
    set_foc_pwm_a(PWM_ARR >> 1);
    set_foc_pwm_b(PWM_ARR >> 1);
    set_foc_pwm_c(PWM_ARR >> 1);
    // 启动PWM输出
    start_foc_pwm_a();
    start_foc_pwm_b();
    start_foc_pwm_c();
<<<<<<< HEAD
=======
    start_foc_pwm_trig();
>>>>>>> 57f1f94 (初次提交FOC代码)
    // 使能DRV8313
    enable_foc_pwm_driver();
}


/**
 * @brief 停止FOC电机PWM输出
 * @param None
 * @retval None
 */
void Foc_Pwm_Stop(void)
{
    // 关闭PWM输出
    stop_foc_pwm_a();
    stop_foc_pwm_b();
    stop_foc_pwm_c();
<<<<<<< HEAD
=======
    stop_foc_pwm_trig();
>>>>>>> 57f1f94 (初次提交FOC代码)
    // 失能DRV8313
    disable_foc_pwm_driver();
}


/**
 * @brief 使FOC电机低侧输出低电平 (用于电机刹车)
 * @param None
 * @retval None
 */
void Foc_Pwm_LowSides(void)
{
    // 设置占空比为 0% (让半桥输出 GND)
    set_foc_pwm_a(0);
    set_foc_pwm_b(0);
    set_foc_pwm_c(0);
    // 启动PWM输出
    start_foc_pwm_a();
    start_foc_pwm_b();
    start_foc_pwm_c();
}


/**
 * @brief 设置FOC电机PWM占空比
 * @param a 通道1占空比 (0-1)
 * @param b 通道2占空比 (0-1)
 * @param c 通道3占空比 (0-1)
 * @retval None
 */
void Foc_Set_Pwm(float a, float b, float c)
{
    set_foc_pwm_a((uint16_t)(a *PWM_ARR));  // 设置通道1的占空比
    set_foc_pwm_b((uint16_t)(b *PWM_ARR));  // 设置通道2的占空比
    set_foc_pwm_c((uint16_t)(c *PWM_ARR));  // 设置通道3的占空比
}
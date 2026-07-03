#ifndef BLDCMOTOR_H
#define BLDCMOTOR_H
#include "main.h"
#include "tim.h"

// 基础配置
#define PWM_ARR (htim1.Init.Period)  						// PWM 自动重装载值用来计算占空比

// 定义宏，用于设置FOC电机的PWM
#define set_foc_pwm_a(value)        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, value)     // 设置A相PWM占空比
#define set_foc_pwm_b(value)        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, value)     // 设置B相PWM占空比
#define set_foc_pwm_c(value)        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, value)     // 设置C相PWM占空比

#define start_foc_pwm_a()           HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);               // 启动A相PWM输出
#define start_foc_pwm_b()           HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);               // 启动B相PWM输出
#define start_foc_pwm_c()           HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);               // 启动C相PWM输出
#define start_foc_pwm_trig()        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);               // 启动ADC触发通道

#define stop_foc_pwm_a()            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);                // 关闭A相PWM输出
#define stop_foc_pwm_b()            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);                // 关闭B相PWM输出
#define stop_foc_pwm_c()            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);                // 关闭C相PWM输出
#define stop_foc_pwm_trig()         HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);                // 关闭ADC触发通道

#define enable_foc_pwm_driver()     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);     // 使能DRV8313
#define disable_foc_pwm_driver()    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);   // 失能DRV8313

void Foc_Pwm_Start(void);
void Foc_Pwm_Stop(void);
void Foc_Pwm_LowSides(void);
void Foc_Set_Pwm(float a, float b, float c);

#endif
// BLDCMOTOR_H

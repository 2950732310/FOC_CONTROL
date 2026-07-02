#ifndef FOC_THREAD_H
#define FOC_THREAD_H

#include "main.h"
#include "FOC.h"
#include "BLDCMotor.h"
#include "mt6816_encoder.h"


/* 电机错误状态枚举 */
typedef enum
{
	M_ERR	= -1,
	M_OK 	= 0,
}MOTOR_ERROR;


/* 电机运行模式枚举 */
typedef enum 
{
	STATE_MODE_IDLE = 0,  // 空闲模式
  STATE_MODE_DETECTING, // 检测模式
  STATE_MODE_RUNNING,   // 运行模式
  STATE_MODE_GUARD,     // 守护模式
}STATE_MODE;


/* 电机闭环类型枚举 */
typedef enum
{
    CONTROL_MODE_OPEN = 0,          // 开环控制模式
    CONTROL_MODE_TORQUE = 1,        // 力矩闭环控制模式
    CONTROL_MODE_VELOCITY = 2,      // 速度闭环控制模式
    CONTROL_MODE_POSITION = 3,      // 位置闭环控制模式
} CONTROL_MODE;



typedef struct
{
	STATE_MODE 		State_Mode;				// 运行状态
	CONTROL_MODE 	Control_Mode;			// 闭环类型
	FOC_DATA 			foc;							// FOC参数结构体
	ENCODER_DATA 	*mt6816;						// 编码器数据结构体
}MOTOR_DATA;

extern MOTOR_DATA motor;

/**
 * @brief FOC 控制函数
 * @param argument: 无参数
 * @retval None
 */
void FOC_Control(void *argument);


/**
 * @brief 电角度零点对齐函数
 * @param motor: MOTOR_DATA 结构体指针
 * @retval None
 */
void FOC_Align(MOTOR_DATA *motor);

/**
 * @brief 电流零点校准函数
 * @param motor: MOTOR_DATA 结构体指针
 * @note  关闭PWM驱动后多次采样ADC取平均，得到电流零点偏移
 *        调用后可通过 motor.foc.i_a_offset / i_b_offset 获取偏移值
 */
void FOC_CurrentOffsetCalibration(MOTOR_DATA *motor);

/**
 * @brief 开环控制函数
 * @param motor: MOTOR_DATA 结构体指针
 * @retval M_OK 	成功
 *					M_ERR 故障
 */
MOTOR_ERROR Open_Loop_Control(MOTOR_DATA *motor);

#endif

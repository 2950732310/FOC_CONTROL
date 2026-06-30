#ifndef MT6816_H
#define MT6816_H

#include "main.h"
#include "spi.h"
#include "stdbool.h"

/**
 * @brief       MT6816 最大延时时间
 */
#define MT6816_MAX_DELAY (10) 

#define SPI_CS					GPIOC
#define SPI_CS_PIN 				GPIO_PIN_11
#define MT6816_SPI_Get_HSPI 	(hspi3) 

#define MT6816_SPI_CS_L() HAL_GPIO_WritePin(SPI_CS, SPI_CS_PIN, GPIO_PIN_RESET)
#define MT6816_SPI_CS_H() HAL_GPIO_WritePin(SPI_CS, SPI_CS_PIN, GPIO_PIN_SET)


typedef struct
{
	uint16_t oriData;					// 原始数据
	float PI_angle;						// 归一化机械角度
	float angle;							// 机械角度（角度制）
	float radian;							// 机械角度（弧度制）
	int dir; 									// 方向
}Encoder_Date;


/**
 * @brief       MT6816 初始化寄存器
 */
#define MT6816_Init_Reg (0x00)              // 初始化寄存器
#define MT6816_Angle_Reg (0x80 | 0x03)      // 角度寄存器
#define MT6816_Warning_Reg (0x80 | 0x04)    // 警告寄存器
#define MT6816_Over_Speed_Reg (0x80 | 0x05) // 过速寄存器


#define M_pi   (3.14159265358979323846f)	    	// 圆周率

/**
 * @brief       MT6816 PLL 带宽
 * */
#define ENCODER_PLL_BANDWIDTH 2000.0f				// PLL 带宽
#define ENCODER_CPR 16384u     							// CPR 值
#define ENCODER_CPR_F 16384.0f 							// CPR 值的浮点数表示
#define ENCODER_CPR_DIV (ENCODER_CPR >> 1)	// 2  CPR 值
#define MAX_ANGLE 360.0f      							// 360 度
#define MAX_ANGLE_HALF 180.0f 							// 180 度

/* MT6816 方向 */
typedef enum
{
    CW = 1,     // 顺时针
    CCW = -1,   // 逆时针
    UNKNOWN = 0 // 未知	
} Direction;


/* SPI DMA 连续采样缓冲区 (4字节 = 1次角度读数) */
#define MT6816_DMA_BUF_SIZE  4

extern volatile uint16_t mt6816_latest_raw;
extern volatile uint8_t  mt6816_data_ready;

void     MT6816_DMA_Start(void);               /* 启动持续DMA采样(只调一次) */
uint16_t mt6816_get_raw(void);                 /* 非阻塞读取最新缓存值      */
bool     GetMotor_Angle(Encoder_Date *encoder); /* 从缓存解析角度            */
bool     GetMotor_Angle(Encoder_Date *encoder);  /* 解析缓存数据到结构体     */
#endif

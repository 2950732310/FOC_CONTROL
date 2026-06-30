#ifndef LED_H
#define LED_H
#include "main.h"

#define LED_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_4

#define LED(x)  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, x)              //LED控制
#define LED_TOGGLE()  LED(!HAL_GPIO_ReadPin(LED_GPIO_Port, LED_Pin))  //LED状态切换

#endif
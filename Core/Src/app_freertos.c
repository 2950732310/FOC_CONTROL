/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "led.h"
#include "flash_thread.h"
#include "serial_thread.h"
<<<<<<< HEAD
#include "instructResponse_thread.h"
=======
>>>>>>> 57f1f94 (初次提交FOC代码)
#include "foc_thread.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for ledThread */
osThreadId_t ledThreadHandle;
const osThreadAttr_t ledThread_attributes = {
  .name = "ledThread",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 128 * 4
};
/* Definitions for flashThread */
osThreadId_t flashThreadHandle;
const osThreadAttr_t flashThread_attributes = {
  .name = "flashThread",
  .priority = (osPriority_t) osPriorityBelowNormal1,
  .stack_size = 256 * 4
};
/* Definitions for serialThread */
osThreadId_t serialThreadHandle;
const osThreadAttr_t serialThread_attributes = {
  .name = "serialThread",
  .priority = (osPriority_t) osPriorityBelowNormal2,
  .stack_size = 512 * 4
};
<<<<<<< HEAD
/* Definitions for instructThread */
osThreadId_t instructThreadHandle;
const osThreadAttr_t instructThread_attributes = {
  .name = "instructThread",
  .priority = (osPriority_t) osPriorityBelowNormal2,
  .stack_size = 256 * 4
};
=======
>>>>>>> 57f1f94 (初次提交FOC代码)
/* Definitions for focControlThrea */
osThreadId_t focControlThreaHandle;
const osThreadAttr_t focControlThrea_attributes = {
  .name = "focControlThrea",
  .priority = (osPriority_t) osPriorityBelowNormal4,
  .stack_size = 512 * 4
};
/* Definitions for flashSem */
osSemaphoreId_t flashSemHandle;
const osSemaphoreAttr_t flashSem_attributes = {
  .name = "flashSem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void led_thread(void *argument);
extern void flash_thread(void *argument);
extern void serial_thread(void *argument);
<<<<<<< HEAD
extern void instructResponse_thread(void *argument);
=======
>>>>>>> 57f1f94 (初次提交FOC代码)
extern void FOC_Control(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of flashSem */
  flashSemHandle = osSemaphoreNew(1, 0, &flashSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ledThread */
  ledThreadHandle = osThreadNew(led_thread, NULL, &ledThread_attributes);

  /* creation of flashThread */
  flashThreadHandle = osThreadNew(flash_thread, NULL, &flashThread_attributes);

  /* creation of serialThread */
  serialThreadHandle = osThreadNew(serial_thread, NULL, &serialThread_attributes);

<<<<<<< HEAD
  /* creation of instructThread */
  instructThreadHandle = osThreadNew(instructResponse_thread, NULL, &instructThread_attributes);

=======
>>>>>>> 57f1f94 (初次提交FOC代码)
  /* creation of focControlThrea */
  focControlThreaHandle = osThreadNew(FOC_Control, NULL, &focControlThrea_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_led_thread */
/**
  * @brief  Function implementing the ledThread thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_led_thread */
void led_thread(void *argument)
{
  /* USER CODE BEGIN led_thread */
  /* Infinite loop */
  for(;;)
  {
    LED_TOGGLE();
    osDelay(500);
  }
  /* USER CODE END led_thread */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


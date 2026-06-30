#ifndef FLASH_THREAD_H
#define FLASH_THREAD_H
#include "main.h"
#include "foc_thread.h"

#define FLASH_ADDR 0x0801F800

/**
 * @brief 存储线程
 * @details 用于存储数据到flash
 * @param param 无
 */
void flash_thread(void *param);

/**
 * @brief 从flash读取数据
 * @details 用于从flash读取数据
 * @param 无
 */
void read_flash_data(void);

#endif

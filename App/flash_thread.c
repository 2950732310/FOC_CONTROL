#include "flash_thread.h"
#include "stmflash.h"
	

void flash_thread(void *param)
{	
	
	while(1)
	{	
		if(osSemaphoreAcquire(flashSemHandle, portMAX_DELAY) == osOK)
		{
			// 存储数据到flash
			stmflash_write(FLASH_ADDR, (uint32_t *)&motor.foc.flash_data, sizeof(motor.foc.flash_data)/4);
		}
        osDelay(10);
	}
}


void read_flash_data(void)
{
	/* 从flash读取数据 */
  stmflash_read(FLASH_ADDR, (uint32_t *)&motor.foc.flash_data, sizeof(motor.foc.flash_data)/4); 
}
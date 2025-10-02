#include "light.h"

/**************************************************************
	@brief:		光电传感器数据读取
	@param:		
	@retval: 		
	@supplement:	仅在前一次没有触发传感器，这一次触发传感器的情
	              况下，反馈数据1.
**************************************************************/
uint8_t Photo_Electricity_Read(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	static uint8_t now_pin, last_pin;
	uint8_t value = 0;
	
	now_pin = HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
	if((now_pin == 1) && (last_pin == 0)) 
	{
		value = 1;
	}
	last_pin = now_pin;
	return value;
}




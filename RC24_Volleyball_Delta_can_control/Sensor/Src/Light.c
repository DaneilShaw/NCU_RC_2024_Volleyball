#include "light.h"

/**************************************************************
	@brief:		��紫�������ݶ�ȡ
	@param:		
	@retval: 		
	@supplement:	����ǰһ��û�д�������������һ�δ�������������
	              ���£���������1.
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




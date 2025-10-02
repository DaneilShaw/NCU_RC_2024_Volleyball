#include "beep.h"

int voice;
void Reset_Beep()
{
	voice = HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_2);
	if (voice != voice)
	{
		
	}
}


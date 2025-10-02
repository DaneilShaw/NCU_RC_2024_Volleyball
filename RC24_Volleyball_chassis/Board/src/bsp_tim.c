#include "bsp_tim.h"

void TIM_Motor_Init(TIM_HandleTypeDef *htim)
{
//	HAL_TIM_Base_Start(htim);//开启定时器
	HAL_TIM_Base_Start_IT(htim);//使能定时器中断
}

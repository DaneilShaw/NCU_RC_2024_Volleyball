#include "bsp_tim.h"

void TIM_Motor_Init(TIM_HandleTypeDef *htim)
{
//	HAL_TIM_Base_Start(htim);//������ʱ��
	HAL_TIM_Base_Start_IT(htim);//ʹ�ܶ�ʱ���ж�
}

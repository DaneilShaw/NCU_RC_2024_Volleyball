#include "TiM_Callback.h"

/**************************************************************
	@brief:		定时控制函数
	@param:		
	@retval: 		
	@supplement:	每隔1ms发送执行一次定时器TIM1中断回调函数
**************************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/*******用于系统计时*******/
	if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
	/***********END***********/
	if(htim == &htim1)
	{
		Refresh_SysTime();
//		switch(GM6020_PIDspd.mode)
//		{
//			case Speed_Mode:
//			{
//				Forwardfeed_Value(&GM6020_PIDspd, GM6020_State_Data.targetspd, 0.001);
//				pid_calc(GM6020_State_Data.targetspd, GM6020_State_Data.spd, &GM6020_PIDspd);
//			} break;
//			
//			case Position_Mode:
//			{
//				pid_calc(GM6020_State_Data.targetpos, GM6020_State_Data.sumpos, &GM6020_PIDpos);
//				Forwardfeed_Value(&GM6020_PIDspd, GM6020_PIDpos.out, 0.001);
//				pid_calc(GM6020_PIDpos.out, GM6020_State_Data.spd, &GM6020_PIDspd);
//			} break;
//			
//			default: break;
//		}
//		Yaw_Current = GM6020_PIDspd.out + GM6020_PIDspd.forwardfeed_value;
//		GM6020_CAN_cmd(voltage2_identifier, Yaw_Current);
	}
}


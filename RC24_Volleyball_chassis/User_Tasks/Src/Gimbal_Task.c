#include "Gimbal_Task.h"

float Inertial_Link_Input(float input, float output, float dt);

//void Gimbal_Task(void const * argument)
//{
//	static float Gimbal_Target_Angle, Forward_Current, Yaw_Current;
//	Get_Basepos(&GM6020_State_Data);
//	TickType_t xLastWakeTime;
//	xLastWakeTime = xTaskGetTickCount();//获取时钟节拍
//	for(;;)
//	{
//		Gimbal_Target_Angle = Inertial_Link_Input(GM6020_State_Data.targetpos, GM6020_State_Data.sumpos, 0.005);
//		pid_calc(Gimbal_Target_Angle, GM6020_State_Data.sumpos, &GM6020_PIDpos);
//		Forward_Current = Forwardfeed_Value(&GM6020_PIDspd, GM6020_PIDpos.out, 0.005);
//		pid_calc(GM6020_PIDpos.out, GM6020_State_Data.spd, &GM6020_PIDspd);
//		Yaw_Current = GM6020_PIDspd.out + Forward_Current;
//		GM6020_CAN_cmd(voltage2_identifier, Yaw_Current);
//		
//		osDelayUntil(&xLastWakeTime, 5);//绝对延时
//	}
//	
//}
/**************************************************************
	@brief:  惯性环节
	@param:		
	@retval: 		
	@supplement: 对阶跃信号连续化
**************************************************************/
float Inertial_Link_Input(float input, float output, float dt)
{
	float T = 0.25;//惯性环节时间常数，越大收敛越慢
	dt = 0.009;
	float update_output;
	update_output = (input * dt + output * (T - dt)) / T;
	return update_output;
}

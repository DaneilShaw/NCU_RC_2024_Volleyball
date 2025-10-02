#include "Gimbal_Task.h"

float Inertial_Link_Input(float input, float output, float dt);

//void Gimbal_Task(void const * argument)
//{
//	static float Gimbal_Target_Angle, Forward_Current, Yaw_Current;
//	Get_Basepos(&GM6020_State_Data);
//	TickType_t xLastWakeTime;
//	xLastWakeTime = xTaskGetTickCount();//��ȡʱ�ӽ���
//	for(;;)
//	{
//		Gimbal_Target_Angle = Inertial_Link_Input(GM6020_State_Data.targetpos, GM6020_State_Data.sumpos, 0.005);
//		pid_calc(Gimbal_Target_Angle, GM6020_State_Data.sumpos, &GM6020_PIDpos);
//		Forward_Current = Forwardfeed_Value(&GM6020_PIDspd, GM6020_PIDpos.out, 0.005);
//		pid_calc(GM6020_PIDpos.out, GM6020_State_Data.spd, &GM6020_PIDspd);
//		Yaw_Current = GM6020_PIDspd.out + Forward_Current;
//		GM6020_CAN_cmd(voltage2_identifier, Yaw_Current);
//		
//		osDelayUntil(&xLastWakeTime, 5);//������ʱ
//	}
//	
//}
/**************************************************************
	@brief:  ���Ի���
	@param:		
	@retval: 		
	@supplement: �Խ�Ծ�ź�������
**************************************************************/
float Inertial_Link_Input(float input, float output, float dt)
{
	float T = 0.25;//���Ի���ʱ�䳣����Խ������Խ��
	dt = 0.009;
	float update_output;
	update_output = (input * dt + output * (T - dt)) / T;
	return update_output;
}

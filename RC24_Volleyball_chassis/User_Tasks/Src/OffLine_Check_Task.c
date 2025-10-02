#include "OffLine_Check_Task.h"
/*****************���ݶ���*****************/
Off_Line_TypeDef Off_Line;
float ChassisDriveTask_Moment = 0.0f;
float Vision_Moment = 0.0f;
float system_time = 0.0f;
/*****************�ֲ�����*****************/
void Off_Line_Init(void);
void Refresh_SysTime(void);
float Get_Systerm_Time(void);
//void Refresh_Task_OffLine_Time(void);
void Task_Offline_Check(void);


/******************************************/
//void OffLine_Check_Task(void const * argument)
//{
//	osDelay(1500);
//	Off_Line_Init();
//	
//	TickType_t xLastWakeTime;
//	xLastWakeTime = xTaskGetTickCount();//��ȡʱ�ӽ���
//	for(;;)
//	{
//		//����ʱ��
//		Task_Offline_Check();
//		
//		if(Off_Line.task_offline_flag & 0x01)
//		{
//			/*******************����ʱ�����е�������֪ͨ*******************/	
//			xTaskNotifyGive(ChassisDrive_Handle);
//			portYIELD();
//		}
//	  else if(Off_Line.task_offline_flag & 0x02)
//		{
//			/*******************����ʱ����������֪ͨ*******************/	
//			xTaskNotifyGive(Vision_Handle);
//			portYIELD();
//		}
//		osDelayUntil(&xLastWakeTime,7);
//	}
//}


/**************************************************************
	@brief:  ���߼���ʼ��
	@param:		
	@retval: 		
	@supplement: 
**************************************************************/
void Off_Line_Init(void)
{
	Off_Line.mode = 0;
	Off_Line.state = 1;
	Off_Line.task = 0;
	Off_Line.time = 0;
	Off_Line.htim = &htim1;
	ChassisDriveTask_Moment = 0.0f;
	Vision_Moment = 0.0f;
}
/**************************************************************
	@brief:  ����ϵͳʱ��ms
	@param:		
	@retval: 		
	@supplement: 
**************************************************************/
void Refresh_SysTime(void)
{
	Off_Line.time += 1;
}
/**************************************************************
	@brief:  ��õ�ǰ��������ʱ��
	@param:		
	@retval: 		
	@supplement: 
**************************************************************/
 float Get_Systerm_Time(void)
{
	return Off_Line.htim->Instance->CNT / 100.0f + Off_Line.time;  //������õ���������ʱʱ��
	/* Instance �� TIM_HandleTypeDef�ṹ���е�һ����Ա */
	/* CNT�Ƕ�ʱ����һ���Ĵ������������˶�ʱ���ĵ�ǰ����ֵ�����ֵ���Ŷ�ʱ�������ж����� */
	/* ��ֵ/100.0f������ʱ������ֵת��Ϊ��msΪ��λ��ʵ��ʱ�� */
}

///**************************************************************
//	@brief:  ˢ������ʱ������
//	@param:		
//	@retval: 		
//	@supplement: 
//**************************************************************/
//void Refresh_Task_OffLine_Time(void)
//{
//	ChassisDriveTask_Moment = Get_Systerm_Time();
//}
/**************************************************************
	@brief:  �������ʱ����
	@param:		
	@retval: 		
	@supplement: 
**************************************************************/
void Task_Offline_Check(void)
{
	system_time = Get_Systerm_Time();//��õ�ǰϵͳʱ��
	if(system_time - ChassisDriveTask_Moment > Offline_Time)
	{
		Off_Line.task_offline_flag = 1;
	}
	else if(system_time - Vision_Moment > Offline_Time)
	{
		Off_Line.task_offline_flag = 2;
	}
	else
	{
		Off_Line.task_offline_flag = 0;
	}
}


#include "OffLine_Check_Task.h"
/*****************数据定义*****************/
Off_Line_TypeDef Off_Line;
float ChassisDriveTask_Moment = 0.0f;
float Vision_Moment = 0.0f;
float system_time = 0.0f;
/*****************局部函数*****************/
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
//	xLastWakeTime = xTaskGetTickCount();//获取时钟节拍
//	for(;;)
//	{
//		//更新时间
//		Task_Offline_Check();
//		
//		if(Off_Line.task_offline_flag & 0x01)
//		{
//			/*******************掉线时，进行底盘任务通知*******************/	
//			xTaskNotifyGive(ChassisDrive_Handle);
//			portYIELD();
//		}
//	  else if(Off_Line.task_offline_flag & 0x02)
//		{
//			/*******************掉线时，进行任务通知*******************/	
//			xTaskNotifyGive(Vision_Handle);
//			portYIELD();
//		}
//		osDelayUntil(&xLastWakeTime,7);
//	}
//}


/**************************************************************
	@brief:  断线检测初始化
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
	@brief:  更新系统时间ms
	@param:		
	@retval: 		
	@supplement: 
**************************************************************/
void Refresh_SysTime(void)
{
	Off_Line.time += 1;
}
/**************************************************************
	@brief:  获得当前任务运行时间
	@param:		
	@retval: 		
	@supplement: 
**************************************************************/
 float Get_Systerm_Time(void)
{
	return Off_Line.htim->Instance->CNT / 100.0f + Off_Line.time;  //经计算得到最终任务即时时间
	/* Instance 是 TIM_HandleTypeDef结构体中的一个成员 */
	/* CNT是定时器的一个寄存器，它包含了定时器的当前计数值，这个值随着定时器的运行而增加 */
	/* 数值/100.0f，将定时器的数值转换为以ms为单位的实际时间 */
}

///**************************************************************
//	@brief:  刷新任务时间数组
//	@param:		
//	@retval: 		
//	@supplement: 
//**************************************************************/
//void Refresh_Task_OffLine_Time(void)
//{
//	ChassisDriveTask_Moment = Get_Systerm_Time();
//}
/**************************************************************
	@brief:  任务断线时间检测
	@param:		
	@retval: 		
	@supplement: 
**************************************************************/
void Task_Offline_Check(void)
{
	system_time = Get_Systerm_Time();//获得当前系统时间
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


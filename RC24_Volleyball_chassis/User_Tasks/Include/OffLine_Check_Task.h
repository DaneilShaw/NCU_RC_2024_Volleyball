#ifndef __OFFLINE_CHECK_TASK_H
#define __OFFLINE_CHECK_TASK_H
#include "My_Includes.h"
#include "Module_Includes.h"

#define Offline_Time 300

typedef struct
{
	uint8_t mode;											//运行模式
	uint8_t state;										//状态
	uint8_t task;											//任务
	uint32_t time;										//System run time mm
	TIM_HandleTypeDef *htim;					//时间计数器句柄
	uint8_t task_offline_flag;				//任务断线标志
}Off_Line_TypeDef;

extern float ChassisDriveTask_Moment;
extern float Vision_Moment;

extern osThreadId ChassisDrive_Handle;
extern osThreadId Vision_Handle;

extern void Off_Line_Init(void);
extern void Refresh_SysTime(void);
extern float Get_Systerm_Time(void);
//extern void Refresh_Task_OffLine_Time(void);
extern void Task_Offline_Check(void);

#endif


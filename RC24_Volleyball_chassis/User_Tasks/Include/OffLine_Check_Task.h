#ifndef __OFFLINE_CHECK_TASK_H
#define __OFFLINE_CHECK_TASK_H
#include "My_Includes.h"
#include "Module_Includes.h"

#define Offline_Time 300

typedef struct
{
	uint8_t mode;											//����ģʽ
	uint8_t state;										//״̬
	uint8_t task;											//����
	uint32_t time;										//System run time mm
	TIM_HandleTypeDef *htim;					//ʱ����������
	uint8_t task_offline_flag;				//������߱�־
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


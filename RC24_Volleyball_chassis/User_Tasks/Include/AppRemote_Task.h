#ifndef __APPREMOTE_TASK_H
#define __APPREMOTE_TASK_H

#include "CushionBall_Task.h"
#include "ChassisDrive_Task.h"

#include "My_Includes.h"
#include "Module_Includes.h"

extern osThreadId AppRemote_Handle;
extern uint8_t Chassis_Mode;

typedef struct
{
	uint8_t cushionball_flag;
	uint8_t appdata_txarm_flag;
}Task_Flag;

extern Task_Flag task_flag;
extern int Cushion_Enable;
extern int Hit_Enable;

#endif


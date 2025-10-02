#ifndef __APPREMOTE_TASK_H
#define __APPREMOTE_TASK_H

#include "CushionBall_Task.h"
#include "wheel_calculation.h"
#include "My_Includes.h"
#include "Module_Includes.h"
#include "HitBall_Task.h"

extern osThreadId AppRemote_Handle;
extern uint8_t Chassis_Mode;
extern int Light_On;


typedef struct
{
	uint8_t cushionball_flag;
	uint8_t hitball_flag;
	uint8_t appdata_txarm_flag;
	uint8_t visiondata_txarm_flag;
}Task_Flag;


extern Task_Flag task_flag;

#endif


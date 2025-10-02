#ifndef __CUSHIONBALL_TASK
#define __CUSHIONBALL_TASK

#include "My_Includes.h"
#include "Module_Includes.h"
#include "ArmDrive_Task.h"
#include "Vision_Receive.h"
#include "AppRemote_Task.h"
#include "light.h"


extern float Vision_KP[3] ;

extern void Arm_Get_Basepos(void);
extern void Delta_frist_hit(void);
extern void Delta_PID_POScalc(DjiMotor_State * ptr, pid_t * pid);
extern void Delta_PID_SPDcalc(DjiMotor_State * ptr, pid_t * pidpos, pid_t * pidspd);

enum
{
	Cushion_To_Vision = 1,
	firstball ,
	up_action, 	//д╛хо	up_action == 2
	down_action,//д╛хо	down_action == 3
	Null,				//д╛хо	Null == 4
};

#endif


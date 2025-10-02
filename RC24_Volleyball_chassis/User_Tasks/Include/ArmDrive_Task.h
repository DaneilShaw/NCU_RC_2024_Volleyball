#ifndef __ARMDRIVE_TASK
#define __ARMDRIVE_TASK

#include "My_Includes.h"
#include "Module_Includes.h"

#define Cushion_IncreTheta2 0.6229f
#define Cushion_IncreTheta3 -0.5301f
#define Cushion_IncreTheta4 -0.5322f//-0.5622f

#define TwoJoint 0
#define ThreeJoint 1
#define FourJoint 2

extern float Arm_Targetpos[3];
extern float Arm_Basepos[3];
extern float Arm_Upos[3];
extern float Arm_Downpos[3];

#endif


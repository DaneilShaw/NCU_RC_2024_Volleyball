#ifndef __MODULE_INCLUDES_H
#define __MODULE_INCLUDES_H

/***************************************感知模块文件***************************************/

#include "briter_encoder.h"
#include "Saber_Inertia.h"
#include "Odometer.h"

/***************************************通信模块文件***************************************/

#include "AppData_Receive.h"
#include "Scope_printf.h"
#include "Vision_Receive.h"
#include "Internal_Communication.h"

/***************************************算法模块文件***************************************/

#include "math_formula.h"
#include "Path_Tracking.h"
#include "pid.h"
#include "wheel_calculation.h"
#include "Path_Data.h"
#include "Filter_Algorithm.h"

/***************************************驱动模块文件***************************************/
#include "HTM_5046.h"
#include "GO_M8010_6.h"
#include "DjiMotor.h"
#include "vesc6.h"
#include "Motor_Feedback.h"
#include "TiM_Callback.h"
#include "GIM4310_steadywin.h"

/***************************************End***************************************/

#endif


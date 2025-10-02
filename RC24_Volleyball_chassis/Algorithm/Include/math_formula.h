#ifndef __MATH_FORMULA_H
#define __MATH_FORMULA_H

#include "math.h"
#include "My_Includes.h"

#define pi 3.1415926f
#define RAD2ANGLE(x) ((x)/pi*180.0f)	//弧度制转为角度值

extern void limit_amplitude(float *a, float *limit);
extern float InvSqrt(float x);
extern int Bubble_Sort(float data[], int n);

#endif

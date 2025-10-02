#ifndef __MATH_FORMULA_H
#define __MATH_FORMULA_H

#include "math.h"

#define pi 3.1415926f
#define RAD2ANGLE(x) ((x)/pi*180.0f)

extern void limit_amplitude(float *a, float *limit);
extern float InvSqrt(float x);
extern int Bubble_Sort(int data[], int n);
extern float absoluteValue(float x);

#endif

#ifndef __WHEEL_CALCULATION_H
#define __WHEEL_CALCULATION_H
#include "math_formula.h"

#define rotation_radius 1    //半径

extern float four_chassis[4];
extern float three_chassis[3];

enum
{
	right_front = 0,
	left_front,
	left_back,
	right_back,
};

typedef struct
{
	float Velvalue;
	float Anglevalue;
}steering_t;

typedef struct
{
	float Vel;
	float Alpha;  //航向角 
	float Target_PosX;
	float Target_PosY;
}World_Coordinate;//世界坐标系下仅包含平动速度，转动速度属于机器坐标系

typedef struct
{
	float velX;
	float velY;
	float velW;
	float Beta;   //偏航角
	
//	float Former_Beta;
//	float Target_Beta;
}Machine_Coordinate;

typedef struct
{
	float wheel1;
	float wheel2;
	float wheel3;
	float wheel4;
}wheel_t;

extern steering_t steer_chassis[3];
extern World_Coordinate Chassis_World;
extern Machine_Coordinate Chassis_Machine;
extern wheel_t Chassis_Wheel;

extern void mecanum_calc(float velX,float velY,float velW);
extern void Chassis_Drive(World_Coordinate * ptr_world, Machine_Coordinate * ptr_machine, wheel_t * ptr_wheel);

#endif


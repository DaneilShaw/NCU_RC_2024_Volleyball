#ifndef __WHEEL_CALCULATION_H
#define __WHEEL_CALCULATION_H
#include "math_formula.h"

#define rotation_radius 1    //�뾶

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
	float Alpha;  //����� 
	float Target_PosX;
	float Target_PosY;
}World_Coordinate;//��������ϵ�½�����ƽ���ٶȣ�ת���ٶ����ڻ�������ϵ

typedef struct
{
	float velX;
	float velY;
	float velW;
	float Beta;   //ƫ����
	
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


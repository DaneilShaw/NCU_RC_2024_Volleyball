#ifndef __PATH_DATA_H
#define __PATH_DATA_H

typedef struct
{
	float theta1;//腰关节
	float theta2;//2关节，id为1
	float theta3;//3关节，id为0
	float theta4;//末端关节
}ArmPos_t;

#define firstball_path_point_num 15
#define path_point_num 50


extern ArmPos_t Hybrid_Arm[path_point_num];
extern ArmPos_t Firstball_Up[firstball_path_point_num];

#endif


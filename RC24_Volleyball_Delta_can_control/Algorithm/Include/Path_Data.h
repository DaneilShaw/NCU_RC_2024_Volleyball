#ifndef __PATH_DATA_H
#define __PATH_DATA_H

typedef struct
{
	float theta1;//���ؽ�
	float theta2;//2�ؽڣ�idΪ1
	float theta3;//3�ؽڣ�idΪ0
	float theta4;//ĩ�˹ؽ�
}ArmPos_t;

#define firstball_path_point_num 15
#define path_point_num 50


extern ArmPos_t Hybrid_Arm[path_point_num];
extern ArmPos_t Firstball_Up[firstball_path_point_num];

#endif


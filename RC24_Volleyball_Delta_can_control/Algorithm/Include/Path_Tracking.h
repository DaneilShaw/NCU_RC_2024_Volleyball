#ifndef __PATH_TRACKING_H
#define ____PATH_TRACKING_H
#include "struct_typedef.h"
#include "Path_Data.h"

typedef struct
{
	volatile float position;                    // λ��x����λmm
  volatile float speed;                 // λ�˦ȣ���λΪ��
}Trapazoid_t;

typedef struct
{
	volatile float x;               //x����
	volatile float y;               //y����
	volatile float tgvel;            //�����ٶ�
	volatile float tgdir;            //���̷���
	volatile float angle_tgvel;      //���ٶ�
}Point_Parameter;//��Ż������ڸ����Ŀ�����

typedef struct
{
	volatile float vel;
	volatile float dir;
	volatile float angle_vel;
}Chassis_State;

typedef struct
{
	volatile float update_x;
	volatile float update_y;
	volatile float target_x;
	volatile float target_y;
}point_t;//���Ԥ���ĸ������ݺ�Ŀ������

extern Trapazoid_t t_plan[4];
  

#endif


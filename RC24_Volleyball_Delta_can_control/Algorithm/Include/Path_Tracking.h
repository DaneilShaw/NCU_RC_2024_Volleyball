#ifndef __PATH_TRACKING_H
#define ____PATH_TRACKING_H
#include "struct_typedef.h"
#include "Path_Data.h"

typedef struct
{
	volatile float position;                    // 位姿x，单位mm
  volatile float speed;                 // 位姿θ，单位为度
}Trapazoid_t;

typedef struct
{
	volatile float x;               //x坐标
	volatile float y;               //y坐标
	volatile float tgvel;            //底盘速度
	volatile float tgdir;            //底盘方向
	volatile float angle_tgvel;      //角速度
}Point_Parameter;//存放机器人在各点的目标参数

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
}point_t;//存放预瞄点的更新数据和目标数据

extern Trapazoid_t t_plan[4];
  

#endif


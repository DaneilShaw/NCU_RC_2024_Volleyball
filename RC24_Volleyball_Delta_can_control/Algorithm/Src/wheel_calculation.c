#include "wheel_calculation.h"
#include "math.h"

World_Coordinate Chassis_World;
Machine_Coordinate Chassis_Machine;
wheel_t Chassis_Wheel;

float four_chassis[4];
float three_chassis[3];

steering_t steer_chassis[3];

/**************************************************************
	@brief:		世界坐标系转为机器坐标系
	@param:		
	@retval: 		
	@supplement:	将底盘宏观的航向角和平移速度转为底盘
	              在自身坐标系下的平移速度
**************************************************************/
void World2Machine_Coordinate_Calc(Machine_Coordinate * ptr_machine, World_Coordinate * ptr_world)
{
	//注意Alpha和Beta角的正负号是否与坐标轴匹配上
	ptr_machine->velX = ptr_world->Vel *  cos(ptr_world->Alpha - ptr_machine->Beta) * pi / 180;
	ptr_machine->velY = ptr_world->Vel *  sin(ptr_world->Alpha - ptr_machine->Beta) * pi / 180;
}

/**************************************************************
	@brief:		麦克纳姆轮正方形解算
	@param:		
	@retval: 		
	@supplement:	
**************************************************************/
void mecanum_calc(float velX,float velY,float velW)//正方形
{
	four_chassis[right_front] = velX + velY + velW*rotation_radius;
  four_chassis[left_front] = velX - velY - velW*rotation_radius;
	four_chassis[left_back] = velX + velY - velW*rotation_radius;
	four_chassis[right_back] = velX - velY + velW*rotation_radius;
}

/**************************************************************
	@brief:		全向轮正三角形和正方形解算
	@param:		
	@retval: 		
	@supplement:	机器坐标系下的算法解算，获得每个轮子的线速度
**************************************************************/
//全向轮正方形
void Omni_wheel4_calc(Machine_Coordinate * ptr)
{
	float motor_radius = pi / 4;
	four_chassis[right_front] = ptr->velX * cos(motor_radius) + ptr->velY * cos(motor_radius) + ptr->velW * rotation_radius;
  four_chassis[left_front] = ptr->velX * cos(motor_radius) - ptr->velY * cos(motor_radius) + ptr->velW * rotation_radius;
	four_chassis[left_back] = -ptr->velX * cos(motor_radius) - ptr->velY  * cos(motor_radius) + ptr->velW *  rotation_radius;
	four_chassis[right_back] = -ptr->velX * cos(motor_radius) + ptr->velY * cos(motor_radius) + ptr->velW * rotation_radius;
}
//全向轮正三角形
void Omni_wheel3_calc(Machine_Coordinate * ptr)//60°
{
	//在确定传感器的数值和电机的方向后，重新算一次，确定正负号
	three_chassis[0] = ptr->velX - ptr->velW * rotation_radius;
	three_chassis[1] = -0.5f * ptr->velX + 0.5f * ptr->velY - ptr->velW * 454;
	three_chassis[2] = -0.5f * ptr->velX - 0.5f * ptr->velY - ptr->velW * 454;
}
/**************************************************************
	@brief:		舵轮正三角形解算
	@param:		
	@retval: 		
	@supplement:	电机速度单位mm/s，注意转角的角度和弧度的转化
**************************************************************/
void steering_calc(float velX, float velY, float velW)
{
	float MVelx,MVely;
	MVelx = velX + velW * rotation_radius;
	MVely = velY;
	steer_chassis[0].Velvalue = sqrt(MVelx*MVelx + MVely*MVely);
	steer_chassis[0].Anglevalue = (atan2(MVely,MVelx))/pi*180;//角度制
	
	MVelx = velX + velW * cos(2 * pi / 3) * rotation_radius;
	MVely = velY + velW * sin(2 * pi / 3) * rotation_radius;
	steer_chassis[1].Velvalue = sqrt(MVelx*MVelx + MVely*MVely);
	steer_chassis[1].Anglevalue = (atan2(MVely,MVelx)) / pi * 180;//角度制
	
	MVelx = velX + velW * cos(-2 * pi / 3) * rotation_radius;
	MVely = velY + velW * sin(-2 * pi / 3) * rotation_radius;
	steer_chassis[2].Velvalue = sqrt(MVelx*MVelx + MVely*MVely);
	steer_chassis[2].Anglevalue = (atan2(MVely,MVelx)) / pi * 180;//角度制
}

/**************************************************************
	@brief:		机器坐标系转为电机转速
	@param:		半径74mm
	@retval: 	
	@supplement:	将mm/s转化为rpm 
**************************************************************/
void Machine2Wheel_Calc(wheel_t * ptr)
{
/**19.2 * 60 * three_chassis[0] / (pi * 2 * 74)**/
	ptr->wheel1 = 576 * three_chassis[0] / (pi * 74);
	ptr->wheel2 = 576 * three_chassis[1] / (pi * 74);
	ptr->wheel3 = 576 * three_chassis[2] / (pi * 74);
}

/**************************************************************
	@brief:	    世界坐标系机器人速度转为电机转速
	@param:		
	@retval: 		
	@supplement:	将机器人目标速度和目标方向转为三个电机的转速
**************************************************************/
void Chassis_Drive(World_Coordinate * ptr_world, Machine_Coordinate * ptr_machine, wheel_t * ptr_wheel)
{
	World2Machine_Coordinate_Calc(ptr_machine, ptr_world);
	Omni_wheel3_calc(ptr_machine);
	Machine2Wheel_Calc(ptr_wheel);
}


/************************************************************一种针对底盘电机转角的控制************************************************************/

//void Omni_wheel3_Poscalc(World_Coordinate * ptr_world, Machine_Coordinate * ptr_machine)
//{
//	three_chassis[0] = -ptr_world->Target_PosX + (ptr_machine->velW) * rotation_radius;
//	three_chassis[1] = ptr->velX * cos(motor_radius) - ptr->velY * sin(motor_radius) + ptr->velW * rotation_radius;
//	three_chassis[2] = ptr->velX * cos(motor_radius) + ptr->velY * sin(motor_radius) + ptr->velW * rotation_radius;
//}



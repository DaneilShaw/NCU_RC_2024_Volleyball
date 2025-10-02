#include "wheel_calculation.h"
#include "math.h"

World_Coordinate Chassis_World;
Machine_Coordinate Chassis_Machine;
wheel_t Chassis_Wheel;

float four_chassis[4];
float three_chassis[3];

steering_t steer_chassis[3];

/**************************************************************
	@brief:		��������ϵתΪ��������ϵ
	@param:		
	@retval: 		
	@supplement:	�����̺�۵ĺ���Ǻ�ƽ���ٶ�תΪ����
					����������ϵ�µ�ƽ���ٶ�
								�õ����� X ��Y �����ϵķ��ٶ�
**************************************************************/
void World2Machine_Coordinate_Calc(Machine_Coordinate * ptr_machine, World_Coordinate * ptr_world)
{
	//ע��Alpha��Beta�ǵ��������Ƿ���������ƥ����
	/*Alpha����� - Beta��ʼƫ���� = ��ǰ��� */
	ptr_machine->velX = ptr_world->Vel *  cos((ptr_world->Alpha - ptr_machine->Beta) * pi / 180);
	ptr_machine->velY = ptr_world->Vel *  sin((ptr_world->Alpha - ptr_machine->Beta) * pi / 180);
}

/**************************************************************
	@brief:		�����ķ�������ν���
	@param:		
	@retval: 		
	@supplement:	
**************************************************************/
void mecanum_calc(float velX,float velY,float velW)//������
{
	four_chassis[right_front] = velX + velY + velW*rotation_radius;
  four_chassis[left_front] = velX - velY - velW*rotation_radius;
	four_chassis[left_back] = velX + velY - velW*rotation_radius;
	four_chassis[right_back] = velX - velY + velW*rotation_radius;
}

/**************************************************************
	@brief:		ȫ�����������κ������ν���
	@param:		
	@retval: 		
	@supplement:	��������ϵ�µ��㷨���㣬���ÿ�����ӵ����ٶ�
**************************************************************/
//ȫ����������
void Omni_wheel4_calc(Machine_Coordinate * ptr)
{
	float motor_radius = pi / 4;
	four_chassis[right_front] = ptr->velX * cos(motor_radius) + ptr->velY * cos(motor_radius) + ptr->velW * rotation_radius;
  four_chassis[left_front] = ptr->velX * cos(motor_radius) - ptr->velY * cos(motor_radius) + ptr->velW * rotation_radius;
	four_chassis[left_back] = -ptr->velX * cos(motor_radius) - ptr->velY  * cos(motor_radius) + ptr->velW *  rotation_radius;
	four_chassis[right_back] = -ptr->velX * cos(motor_radius) + ptr->velY * cos(motor_radius) + ptr->velW * rotation_radius;
}

//ȫ������������
void Omni_wheel3_calc(Machine_Coordinate * ptr)//60��
{
	//��ȷ������������ֵ�͵���ķ����������һ�Σ�ȷ��������
	/*	335.78 ���������ĵ����̼������ĵľ���	*/
	three_chassis[0] = ptr->velX - ptr->velW * 335.78f;
	three_chassis[1] = -0.5f * ptr->velX + 0.866025f * ptr->velY - ptr->velW * 335.78f;
	three_chassis[2] = -0.5f * ptr->velX - 0.866025f * ptr->velY - ptr->velW * 335.78f;
}
/**************************************************************
	@brief:		�����������ν���
	@param:		
	@retval: 		
	@supplement:	����ٶȵ�λmm/s��ע��ת�ǵĽǶȺͻ��ȵ�ת��
**************************************************************/
void steering_calc(float velX, float velY, float velW)
{
	float MVelx,MVely;
	MVelx = velX + velW * rotation_radius;
	MVely = velY;
	steer_chassis[0].Velvalue = sqrt(MVelx*MVelx + MVely*MVely);
	steer_chassis[0].Anglevalue = (atan2(MVely,MVelx))/pi*180;//�Ƕ���
	
	MVelx = velX + velW * cos(2 * pi / 3) * rotation_radius;
	MVely = velY + velW * sin(2 * pi / 3) * rotation_radius;
	steer_chassis[1].Velvalue = sqrt(MVelx*MVelx + MVely*MVely);
	steer_chassis[1].Anglevalue = (atan2(MVely,MVelx)) / pi * 180;//�Ƕ���
	
	MVelx = velX + velW * cos(-2 * pi / 3) * rotation_radius;
	MVely = velY + velW * sin(-2 * pi / 3) * rotation_radius;
	steer_chassis[2].Velvalue = sqrt(MVelx*MVelx + MVely*MVely);
	steer_chassis[2].Anglevalue = (atan2(MVely,MVelx)) / pi * 180;//�Ƕ���
}

/**************************************************************
	@brief:		��������ϵתΪ���ת��
	@param:		�뾶74mm
	@retval: 	
	@supplement:	��mm/sת��Ϊrpm 
**************************************************************/
void Machine2Wheel_Calc(wheel_t * ptr)
{
/**19.2(3508������ٱ�) * 60( ��(S)ת��Ϊ����(min)) * three_chassis[0] / (pi * 2 * 74)**/
	ptr->wheel1 = 576 * three_chassis[0] / (pi * 74);
	ptr->wheel2 = 576 * three_chassis[1] / (pi * 74);
	ptr->wheel3 = 576 * three_chassis[2] / (pi * 74);
}

/**************************************************************
	@brief:	    ��������ϵ�������ٶ�תΪ���ת��
	@param:		
	@retval: 		
	@supplement:	��������Ŀ���ٶȺ�Ŀ�귽��תΪ���������ת��
(&Chassis_World, &Chassis_Machine, &Chassis_Wheel)
**************************************************************/
void Chassis_Drive(World_Coordinate * ptr_world, Machine_Coordinate * ptr_machine, wheel_t * ptr_wheel)
{
	/* �õ����� X , Y �����ϵķ��ٶ�*/
	World2Machine_Coordinate_Calc(ptr_machine, ptr_world);
	/* ����ȫ���ֽ��㣬���ÿ�����ӵ����ٶ� */
	Omni_wheel3_calc(ptr_machine);
	/* ͨ������ֱ���������ٶȽ���Ϊÿ�������ת�٣��õ����̵����Ŀ��ת��*/
	Machine2Wheel_Calc(ptr_wheel);
}


/************************************************************һ����Ե��̵��ת�ǵĿ���************************************************************/

//void Omni_wheel3_Poscalc(World_Coordinate * ptr_world, Machine_Coordinate * ptr_machine)
//{
//	three_chassis[0] = -ptr_world->Target_PosX + (ptr_machine->velW) * rotation_radius;
//	three_chassis[1] = ptr->velX * cos(motor_radius) - ptr->velY * sin(motor_radius) + ptr->velW * rotation_radius;
//	three_chassis[2] = ptr->velX * cos(motor_radius) + ptr->velY * sin(motor_radius) + ptr->velW * rotation_radius;
//}



#include "CushionBall_Task.h"

/*******************�ź�������*******************/
SemaphoreHandle_t ArmDrive_xSemaphore = NULL;//ArmDrive_Task�ź���
/*******************ȫ�ֱ���*******************/	
extern motor_state_s HTM_StateData;
extern Task_Flag task_flag;
int data_point = 0;
/*******************�ֲ�����*******************/	
void ArmTgpos_Set(ArmPos_t * ptr, int point, float pos3);
void Get_ArmNow_Value(float pos1, float pos2, float pos3);
int Arm_Linear_Interpolation(ArmPos_t * ptr, int buf_size, float pos1, float pos2, float pos3);
float Joint4_Angle_Correction(float joint3_pos);

//void CushionBall_Task(void const * argument)
//{
//	ArmDrive_xSemaphore = xSemaphoreCreateBinary();//ArmDrive_Task�ź�������
//	
//	static float Two_Now_Value, Three_Now_Value, Four_Now_Value;
//	
//	TickType_t xLastWakeTime;
//	xLastWakeTime = xTaskGetTickCount();//��ȡʱ�ӽ���
//	for(;;)
//	{
//		/*******************��ȡ����ļ���λ��ֵ*******************/	
//		Two_Now_Value = GO_TwoArm_RecvData.Pos;
//		Three_Now_Value = GO_ThreeArm_RecvData.Pos;
//		Four_Now_Value = HTM_StateData.pos_rad;
//		/*******************����ģʽ�ж�*******************/
//		switch(task_flag.cushionball_flag)
//		{
//			/*******************λ�˳�ʼ��*******************/
//			case pose_init:
//			{
//				if(Arm_Basepos[ThreeJoint] != 0)
//				{
//				/*******************��е����̬��ʼ��*******************/
//				  ArmTgpos_Set(Firstball_Up, 0, Three_Now_Value);
//				/*******************д����*******************/
//				  GoMotor_PosWrite(&Go_TwoArm_send, two_armid, foc_mode, 0.35f, 0.03f, Arm_Targetpos[TwoJoint]);
//	        GoMotor_PosWrite(&Go_ThreeArm_send, three_armid, foc_mode, 0.3f, 0.025f, Arm_Targetpos[ThreeJoint]);
//	        HTMotor_Write(&HTM_StateData, 2.0f, 3.0f, Arm_Targetpos[FourJoint]);
//				}
//			} break;
//			/*******************���һ����*******************/
//			case start_cushion://1
//			{
//			  /*******************�������Ŀ��ֵ��ֵ*******************/
//				ArmTgpos_Set(Firstball_Up, data_point, Three_Now_Value);
//			  /*******************д����*******************/
//			  GoMotor_PosWrite(&Go_TwoArm_send, two_armid, foc_mode, 2.1f, 0.043f, Arm_Targetpos[TwoJoint]);
//			  GoMotor_PosWrite(&Go_ThreeArm_send, three_armid, foc_mode, 2.1f, 0.043f, Arm_Targetpos[ThreeJoint]);
//			  HTMotor_Write(&HTM_StateData, 14.0f, 14.0f, Arm_Targetpos[FourJoint]);
//				/*******************���Բ�ֵ���涨Ŀ���*******************/
////				data_point =  Arm_Linear_Interpolation(Firstball_Up, firstball_path_point_num, Two_Now_Value, Three_Now_Value, Four_Now_Value);
//				/*******************·�����ж�*******************/
//				if(data_point < firstball_path_point_num - 1)
//			  {
//				  data_point++;
//			  }
//				if((Two_Now_Value >= Arm_Downpos[TwoJoint]) && (Three_Now_Value >= Arm_Downpos[ThreeJoint]) && (Four_Now_Value >= Arm_Downpos[FourJoint]))
//				{
////					data_point = path_point_num;
//					task_flag.cushionball_flag = down_action;
//				}
//			} break;
//			/*******************��������*******************/
//			case up_action://2
//			{
//		  /*******************����·��*******************/	
//			/*******************Ŀ��ֵ��ֵ*******************/
//				ArmTgpos_Set(Hybrid_Arm, data_point, Three_Now_Value);
//			/*******************д����*******************/
//			  GoMotor_PosWrite(&Go_TwoArm_send, two_armid, foc_mode, 2.43f, 0.047f, Arm_Targetpos[TwoJoint]);
//			  GoMotor_PosWrite(&Go_ThreeArm_send, three_armid, foc_mode, 2.43f, 0.047f, Arm_Targetpos[ThreeJoint]);
//			  HTMotor_Write(&HTM_StateData, 11.0f, 12.0f, Arm_Targetpos[FourJoint]);
//			/*******************·�����ж�*******************/
//			  if(data_point < path_point_num - 1)
//			  {
//				  data_point ++;
//			  }
//				if((Two_Now_Value >= Arm_Downpos[TwoJoint]) && (Three_Now_Value >= Arm_Downpos[ThreeJoint]) && (Four_Now_Value >= Arm_Downpos[FourJoint]))
//				{
////					data_point = 0;
//					task_flag.cushionball_flag = down_action;
//				}
//		  } break;
//			/*******************�����½�*******************/
//		  case down_action://3
//		  {
//		  /*******************�½�·��*******************/
//			/*******************·�����ж�*******************/
//				if(data_point >= 1)
//				{
//					data_point--;
//				}
////				else if((Two_Now_Value <= Arm_Upos[TwoJoint]) && (Three_Now_Value <= Arm_Upos[ThreeJoint]) && (Four_Now_Value <= Arm_Upos[FourJoint]))
////				{
////					data_point = 0;
////					task_flag.cushionball_flag = up_action;
////				}
//			/*******************Ŀ��ֵ��ֵ*******************/
//				ArmTgpos_Set(Hybrid_Arm, 5, Three_Now_Value);
//			/*******************д����*******************/
//			  GoMotor_PosWrite(&Go_TwoArm_send, two_armid, foc_mode, 0.3f, 0.012f, Arm_Targetpos[TwoJoint]);
//			  GoMotor_PosWrite(&Go_ThreeArm_send, three_armid, foc_mode, 0.3f, 0.012f, Arm_Targetpos[ThreeJoint]);
//			  HTMotor_Write(&HTM_StateData, 4.0f, 4.0f, Arm_Targetpos[FourJoint]);
//		  } break;
//			
//			default: break;
//		}
//		xSemaphoreGive(ArmDrive_xSemaphore);
//		osDelayUntil(&xLastWakeTime, 2);//������ʱ
//	}
//}
/**************************************************************
	@brief:  ��е��Ŀ��λ��ֵ����
	@param:		
	@retval: 		
	@supplement: 
**************************************************************/
void ArmTgpos_Set(ArmPos_t * ptr, int point, float pos3)
{
	float joint4_pos_correct = 0.0f;
	Arm_Targetpos[TwoJoint] = Arm_Basepos[TwoJoint] + Cushion_IncreTheta2 + ptr[point].theta2;
	Arm_Targetpos[ThreeJoint] = Arm_Basepos[ThreeJoint] + Cushion_IncreTheta3 + ptr[point].theta3;
	if((task_flag.cushionball_flag == pose_init) || (task_flag.cushionball_flag == down_action) || ((task_flag.cushionball_flag == 4)))
	{
		joint4_pos_correct = Joint4_Angle_Correction(pos3);//У��ֵ����
	}
	Arm_Targetpos[FourJoint] = Arm_Basepos[FourJoint] - Cushion_IncreTheta4 - ptr[point].theta4 - joint4_pos_correct;
}
/**************************************************************
	@brief:  ��е��ĩ����̬��̬У��
	@param:		
	@retval: 		
	@supplement: 
**************************************************************/
float Joint4_Angle_Correction(float joint3_pos)
{
	float err;
	err = Arm_Targetpos[ThreeJoint] - joint3_pos;
	return err;
}
/**************************************************************
	@brief:  ��е�����Բ�ֵ
	@param:		
	@retval: 		
	@supplement: 
**************************************************************/
int Arm_Linear_Interpolation(ArmPos_t * ptr, int buf_size, float pos1, float pos2, float pos3)
{
	int insert_num[3], next_num;
	/*****************��ֵ�����*****************/
	insert_num[TwoJoint] = (ceil)(100 * (pos1 - Arm_Basepos[TwoJoint] - Cushion_IncreTheta2 - ptr[0].theta2) / (ptr[buf_size - 1].theta2 - ptr[0].theta2));
	insert_num[ThreeJoint] = (ceil)(100 * (pos2 - Arm_Basepos[ThreeJoint] - Cushion_IncreTheta3 - ptr[0].theta3) / (ptr[buf_size - 1].theta3 - ptr[0].theta3));
	insert_num[FourJoint] = (ceil)(100 * (pos3 - Arm_Basepos[FourJoint] + Cushion_IncreTheta4 + ptr[0].theta4) / (ptr[0].theta4 - ptr[buf_size - 1].theta4));//����Ϊ����
	/*****************�������ֵ��*****************/
	next_num = Bubble_Sort(insert_num, 3);
	/*****************�ж��Ƿ�ﵽ��һ����*****************/
	if((insert_num[TwoJoint] <= 0) || (insert_num[ThreeJoint] <= 0) || (insert_num[FourJoint] <= 0))
	{
		next_num = 0;
	}
	/*****************Ŀ����޸ģ��ı�����ٶ�*****************/
	next_num = next_num + 10;
	/*****************�޷���ֵ�㣬��ֹ��ֵ����������С*****************/
	buf_size = buf_size - 1;
	limit_amplitude((float *)&next_num, (float *)&buf_size);
	return next_num;
}



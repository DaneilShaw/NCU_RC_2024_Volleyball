#include "ArmDrive_Task.h"

/*****************ȫ�ֱ���*****************/
float Arm_Targetpos[3];
float Arm_Basepos[3];
float Arm_Upos[3];
float Arm_Downpos[3];

extern motor_state_s HTM_StateData;
extern SemaphoreHandle_t ArmDrive_xSemaphore;
/*****************�ֲ�����*****************/
void Get_Arm_Basepos(void);
//void Arm_Pose_Init(void);
void Get_Arm_Change_Pos(void);
/******************************************/

//void ArmDrive_Task(void const * argument)
//{
//	osDelay(1000);
//	Get_Arm_Basepos();
////	Arm_Pose_Init();
//	Get_Arm_Change_Pos();
//	
//	for(;;)
//	{
//		Refresh_Task_OffLine_Time();//���»�е����������ʱ��

//		if(xSemaphoreTake(ArmDrive_xSemaphore, portMAX_DELAY) == pdTRUE)//�ź������գ�������ʱ��Ӱ�쵽�������ݽ���
//		{
//		  motor_control_pos_val_tqe(0x01, &HTM_StateData);//��λ�ٻ�Ͽ���
//	    motor_read(0x01);//���ݶ�ȡ
//		
//			modify_data(&Go_TwoArm_send);
//			if((ulTaskNotifyTake(pdTRUE, 2)) != 1){};

//			modify_data(&Go_ThreeArm_send);
//			if((ulTaskNotifyTake(pdTRUE, 2)) != 1){};
//		}
//	}
//}

/**************************************************************
	@brief:  ��ȡ�����ؽڵ���ĳ�ʼֵ
	@param:		
	@retval: 		
	@supplement: 
**************************************************************/
void Get_Arm_Basepos(void)
{
	motor_read(0x01);//���ݶ�ȡ
	while(Arm_Basepos[FourJoint] == 0)//�ȴ����ݻ�ȡ
	{
	  Arm_Basepos[FourJoint] = HTM_StateData.pos_rad;
	}
	
	GoMotor_DampWrite(&Go_TwoArm_send, two_armid, foc_mode, 0.1f);//д�����ݲ���
	modify_data(&Go_TwoArm_send);
	while((ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) != 1);//�ȴ���Ϣ��ȡ
	Arm_Basepos[TwoJoint] = GO_TwoArm_RecvData.Pos;
	
	GoMotor_DampWrite(&Go_ThreeArm_send, three_armid, foc_mode, 0.1f);//д�����ݲ���
	modify_data(&Go_ThreeArm_send);
	while((ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) != 1);//�ȴ���Ϣ��ȡ
	Arm_Basepos[ThreeJoint] = GO_ThreeArm_RecvData.Pos;
} 
/**************************************************************
	@brief:  ����λ�ó�ʼ��
	@param:		
	@retval: 		
	@supplement: 
**************************************************************/
void Arm_Pose_Init(void)
{
  /******************���õ��Ŀ��ֵ******************/
	Arm_Targetpos[TwoJoint] = Arm_Basepos[TwoJoint] + Cushion_IncreTheta2 + Firstball_Up[0].theta2;
	Arm_Targetpos[ThreeJoint] = Arm_Basepos[ThreeJoint] + Cushion_IncreTheta3 + Firstball_Up[0].theta3;
  Arm_Targetpos[FourJoint] = Arm_Basepos[FourJoint] - Cushion_IncreTheta4 - Firstball_Up[0].theta4;
	/******************д����******************/
	GoMotor_PosWrite(&Go_TwoArm_send, two_armid, foc_mode, 0.5f, 0.05f, Arm_Targetpos[TwoJoint]);
	GoMotor_PosWrite(&Go_ThreeArm_send, three_armid, foc_mode, 0.5f, 0.045f, Arm_Targetpos[ThreeJoint]);
	HTMotor_Write(&HTM_StateData, 2.0f, 3.0f, Arm_Targetpos[FourJoint]);
}
/**************************************************************
	@brief:  �����½�λ�ý���仯
	@param:		
	@retval: 		
	@supplement: 
**************************************************************/
void Get_Arm_Change_Pos(void)
{
	/**********************************��е�ۿ�ʼ����**********************************/
	Arm_Upos[TwoJoint] = Arm_Basepos[TwoJoint] + Cushion_IncreTheta2 + Hybrid_Arm[2].theta2;
	Arm_Upos[ThreeJoint] = Arm_Basepos[ThreeJoint] + Cushion_IncreTheta3 + Hybrid_Arm[2].theta3;
	Arm_Upos[FourJoint] = Arm_Basepos[FourJoint] - Cushion_IncreTheta4 - Hybrid_Arm[2].theta4;
	/**********************************��е�ۿ�ʼ�½�**********************************/
	Arm_Downpos[TwoJoint] = Arm_Basepos[TwoJoint] + Cushion_IncreTheta2 + Hybrid_Arm[path_point_num - 2].theta2;
	Arm_Downpos[ThreeJoint] = Arm_Basepos[ThreeJoint] + Cushion_IncreTheta3 + Hybrid_Arm[path_point_num - 2].theta3;
	Arm_Downpos[FourJoint] = Arm_Basepos[FourJoint] - Cushion_IncreTheta4 - Hybrid_Arm[path_point_num - 2].theta4;
}


#include "ArmDrive_Task.h"

/*****************全局变量*****************/
float Arm_Targetpos[3];
float Arm_Basepos[3];
float Arm_Upos[3];
float Arm_Downpos[3];

extern motor_state_s HTM_StateData;
extern SemaphoreHandle_t ArmDrive_xSemaphore;
/*****************局部函数*****************/
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
//		Refresh_Task_OffLine_Time();//更新机械臂驱动任务时间

//		if(xSemaphoreTake(ArmDrive_xSemaphore, portMAX_DELAY) == pdTRUE)//信号量接收，绝对延时会影响到总线数据接收
//		{
//		  motor_control_pos_val_tqe(0x01, &HTM_StateData);//力位速混合控制
//	    motor_read(0x01);//数据读取
//		
//			modify_data(&Go_TwoArm_send);
//			if((ulTaskNotifyTake(pdTRUE, 2)) != 1){};

//			modify_data(&Go_ThreeArm_send);
//			if((ulTaskNotifyTake(pdTRUE, 2)) != 1){};
//		}
//	}
//}

/**************************************************************
	@brief:  获取三个关节电机的初始值
	@param:		
	@retval: 		
	@supplement: 
**************************************************************/
void Get_Arm_Basepos(void)
{
	motor_read(0x01);//数据读取
	while(Arm_Basepos[FourJoint] == 0)//等待数据获取
	{
	  Arm_Basepos[FourJoint] = HTM_StateData.pos_rad;
	}
	
	GoMotor_DampWrite(&Go_TwoArm_send, two_armid, foc_mode, 0.1f);//写入数据参数
	modify_data(&Go_TwoArm_send);
	while((ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) != 1);//等待信息获取
	Arm_Basepos[TwoJoint] = GO_TwoArm_RecvData.Pos;
	
	GoMotor_DampWrite(&Go_ThreeArm_send, three_armid, foc_mode, 0.1f);//写入数据参数
	modify_data(&Go_ThreeArm_send);
	while((ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) != 1);//等待信息获取
	Arm_Basepos[ThreeJoint] = GO_ThreeArm_RecvData.Pos;
} 
/**************************************************************
	@brief:  宇树位置初始化
	@param:		
	@retval: 		
	@supplement: 
**************************************************************/
void Arm_Pose_Init(void)
{
  /******************设置电机目标值******************/
	Arm_Targetpos[TwoJoint] = Arm_Basepos[TwoJoint] + Cushion_IncreTheta2 + Firstball_Up[0].theta2;
	Arm_Targetpos[ThreeJoint] = Arm_Basepos[ThreeJoint] + Cushion_IncreTheta3 + Firstball_Up[0].theta3;
  Arm_Targetpos[FourJoint] = Arm_Basepos[FourJoint] - Cushion_IncreTheta4 - Firstball_Up[0].theta4;
	/******************写入电机******************/
	GoMotor_PosWrite(&Go_TwoArm_send, two_armid, foc_mode, 0.5f, 0.05f, Arm_Targetpos[TwoJoint]);
	GoMotor_PosWrite(&Go_ThreeArm_send, three_armid, foc_mode, 0.5f, 0.045f, Arm_Targetpos[ThreeJoint]);
	HTMotor_Write(&HTM_StateData, 2.0f, 3.0f, Arm_Targetpos[FourJoint]);
}
/**************************************************************
	@brief:  上升下降位置交替变化
	@param:		
	@retval: 		
	@supplement: 
**************************************************************/
void Get_Arm_Change_Pos(void)
{
	/**********************************机械臂开始上升**********************************/
	Arm_Upos[TwoJoint] = Arm_Basepos[TwoJoint] + Cushion_IncreTheta2 + Hybrid_Arm[2].theta2;
	Arm_Upos[ThreeJoint] = Arm_Basepos[ThreeJoint] + Cushion_IncreTheta3 + Hybrid_Arm[2].theta3;
	Arm_Upos[FourJoint] = Arm_Basepos[FourJoint] - Cushion_IncreTheta4 - Hybrid_Arm[2].theta4;
	/**********************************机械臂开始下降**********************************/
	Arm_Downpos[TwoJoint] = Arm_Basepos[TwoJoint] + Cushion_IncreTheta2 + Hybrid_Arm[path_point_num - 2].theta2;
	Arm_Downpos[ThreeJoint] = Arm_Basepos[ThreeJoint] + Cushion_IncreTheta3 + Hybrid_Arm[path_point_num - 2].theta3;
	Arm_Downpos[FourJoint] = Arm_Basepos[FourJoint] - Cushion_IncreTheta4 - Hybrid_Arm[path_point_num - 2].theta4;
}


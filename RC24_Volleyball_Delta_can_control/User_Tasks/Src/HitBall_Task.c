#include "HitBall_Task.h"
/**********************信号量**********************/
SemaphoreHandle_t ArmDrive_xSemaphore = NULL;
/**********************全局变量**********************/
uint8_t To_Vision1 [3] = {"hit"}; 
uint8_t To_Vision2 [3] = {'0','0','0'};
float Tail_Arm_Basepos;
/**********************全局函数**********************/
void Get_GO1_Arm_Basepos(void);
void Hit_Arm_Setpos(void);
extern void Delta_PID_POScalc(DjiMotor_State * ptr, pid_t * pid);
extern void Delta_PID_SPDcalc(DjiMotor_State * ptr, pid_t * pidpos, pid_t * pidspd);

void HitBall_Task(void const * argument)
{
	ArmDrive_xSemaphore = xSemaphoreCreateBinary();//ArmDrive_Task信号量创建
	
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();//获取时钟节拍
	for(;;)
	{
		switch(task_flag.hitball_flag)
		{
			case Serve_To_Vision:
			{
				HAL_UART_Transmit(&huart2, To_Vision1, 3, 10);
				HAL_Delay(800);
				task_flag.hitball_flag = Arm_UP;	
			} break;
			case Arm_UP:
			{
				Hit_Arm_Setpos();//给定第一次垫球的高度值
				Delta_PID_POScalc(M3508_State_Data, M3508_PID_Hit_UPpos);
				Delta_PID_SPDcalc(M3508_State_Data, M3508_PID_Hit_UPpos, M3508_PID_spd);
//				/*******************调试时使用*******************/
//			steps.serve_flag = Tail_Hit;
				/*******************正常代码********************/
				if(M3508_State_Data[0].sumpos > 35000)
				{				
					task_flag.hitball_flag = Arm_down;
				}
			} break;
			case Arm_down:
			{
				Hit_Arm_Setpos();
				Delta_PID_POScalc(M3508_State_Data, M3508_PID_Hit_Downpos);
				Delta_PID_SPDcalc(M3508_State_Data, M3508_PID_Hit_Downpos, M3508_PID_spd);
				if(M3508_State_Data[0].sumpos < 22000)
				{
					/*通信部分*/
				  task_flag.hitball_flag = Tail_Hit;
				}
			} break;
			case Tail_Hit:
			{
				/*为了将击球平面维持在相对应的高度*/
				Hit_Arm_Setpos();
				Delta_PID_POScalc(M3508_State_Data, M3508_PID_Hit_Downpos);
				Delta_PID_SPDcalc(M3508_State_Data, M3508_PID_Hit_Downpos, M3508_PID_spd);
				
				/*接收视觉数据，并开始击球动作*/
				if( VisionRx_DataBuf[0] == 'H' && VisionRx_DataBuf[1] == 'I' && VisionRx_DataBuf[2] == 'T' )
				{
					//位置控制
//				GoMotor_PosWrite(&Go_TailArm_send, tail_armid, foc_mode, 10.0f, 0.053f,Tail_Arm_tgpos);
					//力矩控制
					GoMotor_ToqWrite(&Go_TailArm_send, tail_armid, foc_mode, 3.5f);
					HAL_UART_Transmit(&huart2,To_Vision2, 3, 10);
				}
				/*回程判断，判断是否回程*/
				if(GO_TailArm_RecvData.Pos > Tail_Arm_Basepos + pi - 0.4f)
				{
					task_flag.hitball_flag = Tail_Reposition;
				}
			} break;
			case Tail_Reposition:
			{
				/*为了将击球平面维持在相对应的高度*/
				Hit_Arm_Setpos();
				Delta_PID_POScalc(M3508_State_Data, M3508_PID_Hit_Downpos);
				Delta_PID_SPDcalc(M3508_State_Data, M3508_PID_Hit_Downpos, M3508_PID_spd);
				/*宇树电机回程位置控制回程*/
				GoMotor_PosWrite(&Go_TailArm_send, tail_armid, foc_mode, 0.3f, 0.053f,Tail_Arm_Basepos);
				memset(VisionRx_DataBuf, 0, VisionRx_Size);
			} break;
			default: break;
		}
		M3508_CAN_cmd(0x200, M3508_PID_spd[0].out, M3508_PID_spd[1].out, M3508_PID_spd[2].out, M3508_PID_spd[3].out);
		xSemaphoreGive(ArmDrive_xSemaphore);
		
		osDelayUntil(&xLastWakeTime, 5);
	}
}
/**************************************************************
	@brief:  获取三个关节电机的初始值
	@param:		
	@retval: 		
	@supplement: 
**************************************************************/
void Get_GO1_Arm_Basepos(void)
{
	GoMotor_DampWrite(&Go_TailArm_send, tail_armid, foc_mode, 0.1f);//写入数据参数
	modify_data(&Go_TailArm_send);
	while((ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) != 1);//等待信息获取
	Tail_Arm_Basepos = GO_TailArm_RecvData.Pos;
}
/**************************************************************
	@brief:  电机位置值设定
	@param:		
	@retval: 		
	@supplement: 
**************************************************************/
void Hit_Arm_Setpos(void)
{
	if(task_flag.hitball_flag == Arm_UP)
	{
	  /*7-7号参数*/
	  M3508_State_Data[0].targetpos = 42000;
	  M3508_State_Data[1].targetpos = 42000;
	  M3508_State_Data[2].targetpos = 42000;
	  M3508_State_Data[3].targetpos = 42000;
	}
	if(task_flag.hitball_flag == Arm_down)
	{
	  M3508_State_Data[0].targetpos = 20000;
	  M3508_State_Data[1].targetpos = 20000;
	  M3508_State_Data[2].targetpos = 20000;
	  M3508_State_Data[3].targetpos = 20000;
	}
}


#include "Serve_Task.h"

/*宇树电机目标值*/
#define Tail_Arm_tgpos 3.2f

/*由于机械限位，3508电机转子最大值 52000,超过52000损伤击球平面*/
#define Serve_frist_UP 30000
#define Serve_frist_Down 28000

SemaphoreHandle_t ArmDrive_xSemaphore = NULL;//ArmDrive_Task信号量

Serve_Task_Flag steps;
uint8_t To_Vision1 [4] = {'h','i','t'}; 
uint8_t To_Vision2 [4] = {'0','0','0'}; 
float Tail_Arm_Basepos;
/************************全局函数*************************/
void Get_GO1_Arm_Basepos(void);
void Arm_Serve_PID_Init	(void);
void Delta_Serve_frist_hit(void);
void Tail_Reposition_Judge(void);
void Delta_Serve_frist_Down(void);
void Current_Set_Zero(void);

float time, last_time;

void Serve_Task(void const * argument)
 {
	 ArmDrive_xSemaphore = xSemaphoreCreateBinary();//ArmDrive_Task信号量创建
	/*DJI 3508电机初始位置*/
	 Arm_Get_Basepos();
	/*3508电机PID初始化*/
	 Arm_Serve_PID_Init();

	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();//获取时钟节拍
	for(;;)
	{
		time = HAL_GetTick() - last_time;
		last_time = HAL_GetTick();
		switch (steps.serve_flag)
		{
			case Serve_To_Vision:
			{
				/*将数据发送给视觉,等待视觉进入快速响应状态*/
				HAL_UART_Transmit(&huart2,To_Vision1,3,HAL_MAX_DELAY);
				HAL_Delay(500);
				steps.serve_flag = Arm_UP;		
			}
			/*第一部分*/
				case Arm_UP:
			{
				Delta_Serve_frist_hit();//给定第一次垫球的高度值
				Delta_PID_POScalc(M3508_State_Data, M3508_PID_Serve_UPpos);
				Delta_PID_SPDcalc(M3508_State_Data, M3508_PID_Serve_UPpos, M3508_PID_spd);
//				/*******************调试时使用*******************/
//			steps.serve_flag = Tail_Hit;
				/*******************正常代码********************/
				if(M3508_State_Data[0].sumpos > Serve_frist_UP - 8000)
				{				
					steps.serve_flag = Arm_down;
				}
			}	break;
//		
				case Arm_down:
				{
					Delta_Serve_frist_Down();//给定第一次回程的高度值
					Delta_PID_POScalc(M3508_State_Data, M3508_PID_Serve_Downpos);
					Delta_PID_SPDcalc(M3508_State_Data, M3508_PID_Serve_Downpos, M3508_PID_spd);
					if(M3508_State_Data[0].sumpos < Serve_frist_Down )
						{
							/*直接将3508输入电流设置为0*/
							Current_Set_Zero();
							/*通信部分*/
							steps.serve_flag = Tail_Hit;
						}
				}break;
//			
			/*驱动宇树电机旋转将球击出*/
				case Tail_Hit:
			{
				/*接收视觉数据，并开始击球动作*/
				if( VisionRx_DataBuf[0] == 'H' && VisionRx_DataBuf[1] == 'I' && VisionRx_DataBuf[2] == 'T' )
				{
					//位置控制
//				GoMotor_PosWrite(&Go_TailArm_send, tail_armid, foc_mode, 10.0f, 0.053f,Tail_Arm_tgpos);
					//力矩控制
					GoMotor_ToqWrite(&Go_TailArm_send, tail_armid, foc_mode, 3.5f);
					HAL_UART_Transmit(&huart2,To_Vision2,3,HAL_MAX_DELAY);
				}
				
				/*回程判断，判断是否回程*/
				if(GO_TailArm_RecvData.Pos > Tail_Arm_tgpos - 0.5f)
				{
					steps.serve_flag = Tail_Reposition;
				}
			} break;
			
			/*宇树电机回程,并清除视觉发送的数据*/	
			 case Tail_Reposition:
			{
				GoMotor_PosWrite(&Go_TailArm_send, tail_armid, foc_mode, 0.3f, 0.053f,Tail_Arm_Basepos);
				memset(VisionRx_DataBuf, 0, VisionRx_Size);
			}
				default: break;	
		}
		
		
		/*避免宇树电机疯转，保护机械*/
		if(GO_TailArm_RecvData.MError != 0)
		{
			GO_TailArm_RecvData.mode = 0;
		}
//		xSemaphoreGive(ArmDrive_xSemaphore);
		/*将数据发送给3508电机,这里的发送函数被注释掉，直接用垫球任务的发送函数*/		
		M3508_CAN_cmd(0x200, M3508_PID_spd[0].out, M3508_PID_spd[1].out, M3508_PID_spd[2].out, M3508_PID_spd[3].out);
		osDelayUntil(&xLastWakeTime, 5);//绝对延时,其它注意不要出现偶数延时，否则其它任务无法获取CPU权限
	}
}
 
/**************************************************************
	@brief:  PID参数设置
	@param:		
	@retval: 		
	@supplement: 
**************************************************************/
void Arm_Serve_PID_Init	(void)
{
		/*向上运动位置环PID初始化*/
	pid_init(&M3508_PID_Serve_UPpos[0], Position_Mode, 0.50f , 0.0f, 0.0f, 0, 8000);
	pid_init(&M3508_PID_Serve_UPpos[1], Position_Mode, 0.50f , 0.0f, 0.0f, 0, 8000);
	pid_init(&M3508_PID_Serve_UPpos[2], Position_Mode, 0.50f , 0.0f, 0.0f, 0, 8000);
	pid_init(&M3508_PID_Serve_UPpos[3], Position_Mode, 0.50f , 0.0f, 0.0f, 0, 8000);
	
		/*向下运动位置环PID初始化*/	
	pid_init(&M3508_PID_Serve_Downpos[0], Position_Mode, 0.0100f , 0.002f, 0.001f, 5000, 6000);
	pid_init(&M3508_PID_Serve_Downpos[1], Position_Mode, 0.0100f , 0.002f, 0.001f, 5000, 6000);
	pid_init(&M3508_PID_Serve_Downpos[2], Position_Mode, 0.0100f , 0.002f, 0.001f, 5000, 6000);
	pid_init(&M3508_PID_Serve_Downpos[3], Position_Mode, 0.0100f , 0.002f, 0.001f, 5000, 6000);
}

/**************************************************************
	@brief:  并联臂将球抛起，目标位置设定
	@param:		
	@retval: 		
	@supplement: 
**************************************************************/
void Delta_Serve_frist_hit(void)
{
	M3508_State_Data[0].targetpos = Serve_frist_UP;
	M3508_State_Data[1].targetpos = Serve_frist_UP;
	M3508_State_Data[2].targetpos = Serve_frist_UP;
	M3508_State_Data[3].targetpos = Serve_frist_UP;
}

/**************************************************************
	@brief:  并联臂将球抛起，目标位置设定
	@param:		
	@retval: 		
	@supplement: 
**************************************************************/
void Current_Set_Zero(void)
{
	M3508_PID_spd[0].out = 0;
	M3508_PID_spd[1].out = 0;
	M3508_PID_spd[2].out = 0;
	M3508_PID_spd[3].out = 0;
}

/**************************************************************
	@brief:  并联臂将球抛起，目标位置设定
	@param:		
	@retval: 		
	@supplement: 
**************************************************************/
void Delta_Serve_frist_Down(void)
{
	M3508_State_Data[0].targetpos = Serve_frist_Down;
	M3508_State_Data[1].targetpos = Serve_frist_Down;
	M3508_State_Data[2].targetpos = Serve_frist_Down;
	M3508_State_Data[3].targetpos = Serve_frist_Down;
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

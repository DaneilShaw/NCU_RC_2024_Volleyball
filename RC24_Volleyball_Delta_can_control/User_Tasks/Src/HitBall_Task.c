#include "HitBall_Task.h"
/**********************�ź���**********************/
SemaphoreHandle_t ArmDrive_xSemaphore = NULL;
/**********************ȫ�ֱ���**********************/
uint8_t To_Vision1 [3] = {"hit"}; 
uint8_t To_Vision2 [3] = {'0','0','0'};
float Tail_Arm_Basepos;
/**********************ȫ�ֺ���**********************/
void Get_GO1_Arm_Basepos(void);
void Hit_Arm_Setpos(void);
extern void Delta_PID_POScalc(DjiMotor_State * ptr, pid_t * pid);
extern void Delta_PID_SPDcalc(DjiMotor_State * ptr, pid_t * pidpos, pid_t * pidspd);

void HitBall_Task(void const * argument)
{
	ArmDrive_xSemaphore = xSemaphoreCreateBinary();//ArmDrive_Task�ź�������
	
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();//��ȡʱ�ӽ���
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
				Hit_Arm_Setpos();//������һ�ε���ĸ߶�ֵ
				Delta_PID_POScalc(M3508_State_Data, M3508_PID_Hit_UPpos);
				Delta_PID_SPDcalc(M3508_State_Data, M3508_PID_Hit_UPpos, M3508_PID_spd);
//				/*******************����ʱʹ��*******************/
//			steps.serve_flag = Tail_Hit;
				/*******************��������********************/
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
					/*ͨ�Ų���*/
				  task_flag.hitball_flag = Tail_Hit;
				}
			} break;
			case Tail_Hit:
			{
				/*Ϊ�˽�����ƽ��ά�������Ӧ�ĸ߶�*/
				Hit_Arm_Setpos();
				Delta_PID_POScalc(M3508_State_Data, M3508_PID_Hit_Downpos);
				Delta_PID_SPDcalc(M3508_State_Data, M3508_PID_Hit_Downpos, M3508_PID_spd);
				
				/*�����Ӿ����ݣ�����ʼ������*/
				if( VisionRx_DataBuf[0] == 'H' && VisionRx_DataBuf[1] == 'I' && VisionRx_DataBuf[2] == 'T' )
				{
					//λ�ÿ���
//				GoMotor_PosWrite(&Go_TailArm_send, tail_armid, foc_mode, 10.0f, 0.053f,Tail_Arm_tgpos);
					//���ؿ���
					GoMotor_ToqWrite(&Go_TailArm_send, tail_armid, foc_mode, 3.5f);
					HAL_UART_Transmit(&huart2,To_Vision2, 3, 10);
				}
				/*�س��жϣ��ж��Ƿ�س�*/
				if(GO_TailArm_RecvData.Pos > Tail_Arm_Basepos + pi - 0.4f)
				{
					task_flag.hitball_flag = Tail_Reposition;
				}
			} break;
			case Tail_Reposition:
			{
				/*Ϊ�˽�����ƽ��ά�������Ӧ�ĸ߶�*/
				Hit_Arm_Setpos();
				Delta_PID_POScalc(M3508_State_Data, M3508_PID_Hit_Downpos);
				Delta_PID_SPDcalc(M3508_State_Data, M3508_PID_Hit_Downpos, M3508_PID_spd);
				/*��������س�λ�ÿ��ƻس�*/
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
	@brief:  ��ȡ�����ؽڵ���ĳ�ʼֵ
	@param:		
	@retval: 		
	@supplement: 
**************************************************************/
void Get_GO1_Arm_Basepos(void)
{
	GoMotor_DampWrite(&Go_TailArm_send, tail_armid, foc_mode, 0.1f);//д�����ݲ���
	modify_data(&Go_TailArm_send);
	while((ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) != 1);//�ȴ���Ϣ��ȡ
	Tail_Arm_Basepos = GO_TailArm_RecvData.Pos;
}
/**************************************************************
	@brief:  ���λ��ֵ�趨
	@param:		
	@retval: 		
	@supplement: 
**************************************************************/
void Hit_Arm_Setpos(void)
{
	if(task_flag.hitball_flag == Arm_UP)
	{
	  /*7-7�Ų���*/
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


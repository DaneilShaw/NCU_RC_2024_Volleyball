#include "Serve_Task.h"

/*�������Ŀ��ֵ*/
#define Tail_Arm_tgpos 3.2f

/*���ڻ�е��λ��3508���ת�����ֵ 52000,����52000���˻���ƽ��*/
#define Serve_frist_UP 30000
#define Serve_frist_Down 28000

SemaphoreHandle_t ArmDrive_xSemaphore = NULL;//ArmDrive_Task�ź���

Serve_Task_Flag steps;
uint8_t To_Vision1 [4] = {'h','i','t'}; 
uint8_t To_Vision2 [4] = {'0','0','0'}; 
float Tail_Arm_Basepos;
/************************ȫ�ֺ���*************************/
void Get_GO1_Arm_Basepos(void);
void Arm_Serve_PID_Init	(void);
void Delta_Serve_frist_hit(void);
void Tail_Reposition_Judge(void);
void Delta_Serve_frist_Down(void);
void Current_Set_Zero(void);

float time, last_time;

void Serve_Task(void const * argument)
 {
	 ArmDrive_xSemaphore = xSemaphoreCreateBinary();//ArmDrive_Task�ź�������
	/*DJI 3508�����ʼλ��*/
	 Arm_Get_Basepos();
	/*3508���PID��ʼ��*/
	 Arm_Serve_PID_Init();

	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();//��ȡʱ�ӽ���
	for(;;)
	{
		time = HAL_GetTick() - last_time;
		last_time = HAL_GetTick();
		switch (steps.serve_flag)
		{
			case Serve_To_Vision:
			{
				/*�����ݷ��͸��Ӿ�,�ȴ��Ӿ����������Ӧ״̬*/
				HAL_UART_Transmit(&huart2,To_Vision1,3,HAL_MAX_DELAY);
				HAL_Delay(500);
				steps.serve_flag = Arm_UP;		
			}
			/*��һ����*/
				case Arm_UP:
			{
				Delta_Serve_frist_hit();//������һ�ε���ĸ߶�ֵ
				Delta_PID_POScalc(M3508_State_Data, M3508_PID_Serve_UPpos);
				Delta_PID_SPDcalc(M3508_State_Data, M3508_PID_Serve_UPpos, M3508_PID_spd);
//				/*******************����ʱʹ��*******************/
//			steps.serve_flag = Tail_Hit;
				/*******************��������********************/
				if(M3508_State_Data[0].sumpos > Serve_frist_UP - 8000)
				{				
					steps.serve_flag = Arm_down;
				}
			}	break;
//		
				case Arm_down:
				{
					Delta_Serve_frist_Down();//������һ�λس̵ĸ߶�ֵ
					Delta_PID_POScalc(M3508_State_Data, M3508_PID_Serve_Downpos);
					Delta_PID_SPDcalc(M3508_State_Data, M3508_PID_Serve_Downpos, M3508_PID_spd);
					if(M3508_State_Data[0].sumpos < Serve_frist_Down )
						{
							/*ֱ�ӽ�3508�����������Ϊ0*/
							Current_Set_Zero();
							/*ͨ�Ų���*/
							steps.serve_flag = Tail_Hit;
						}
				}break;
//			
			/*�������������ת�������*/
				case Tail_Hit:
			{
				/*�����Ӿ����ݣ�����ʼ������*/
				if( VisionRx_DataBuf[0] == 'H' && VisionRx_DataBuf[1] == 'I' && VisionRx_DataBuf[2] == 'T' )
				{
					//λ�ÿ���
//				GoMotor_PosWrite(&Go_TailArm_send, tail_armid, foc_mode, 10.0f, 0.053f,Tail_Arm_tgpos);
					//���ؿ���
					GoMotor_ToqWrite(&Go_TailArm_send, tail_armid, foc_mode, 3.5f);
					HAL_UART_Transmit(&huart2,To_Vision2,3,HAL_MAX_DELAY);
				}
				
				/*�س��жϣ��ж��Ƿ�س�*/
				if(GO_TailArm_RecvData.Pos > Tail_Arm_tgpos - 0.5f)
				{
					steps.serve_flag = Tail_Reposition;
				}
			} break;
			
			/*��������س�,������Ӿ����͵�����*/	
			 case Tail_Reposition:
			{
				GoMotor_PosWrite(&Go_TailArm_send, tail_armid, foc_mode, 0.3f, 0.053f,Tail_Arm_Basepos);
				memset(VisionRx_DataBuf, 0, VisionRx_Size);
			}
				default: break;	
		}
		
		
		/*�������������ת��������е*/
		if(GO_TailArm_RecvData.MError != 0)
		{
			GO_TailArm_RecvData.mode = 0;
		}
//		xSemaphoreGive(ArmDrive_xSemaphore);
		/*�����ݷ��͸�3508���,����ķ��ͺ�����ע�͵���ֱ���õ�������ķ��ͺ���*/		
		M3508_CAN_cmd(0x200, M3508_PID_spd[0].out, M3508_PID_spd[1].out, M3508_PID_spd[2].out, M3508_PID_spd[3].out);
		osDelayUntil(&xLastWakeTime, 5);//������ʱ,����ע�ⲻҪ����ż����ʱ���������������޷���ȡCPUȨ��
	}
}
 
/**************************************************************
	@brief:  PID��������
	@param:		
	@retval: 		
	@supplement: 
**************************************************************/
void Arm_Serve_PID_Init	(void)
{
		/*�����˶�λ�û�PID��ʼ��*/
	pid_init(&M3508_PID_Serve_UPpos[0], Position_Mode, 0.50f , 0.0f, 0.0f, 0, 8000);
	pid_init(&M3508_PID_Serve_UPpos[1], Position_Mode, 0.50f , 0.0f, 0.0f, 0, 8000);
	pid_init(&M3508_PID_Serve_UPpos[2], Position_Mode, 0.50f , 0.0f, 0.0f, 0, 8000);
	pid_init(&M3508_PID_Serve_UPpos[3], Position_Mode, 0.50f , 0.0f, 0.0f, 0, 8000);
	
		/*�����˶�λ�û�PID��ʼ��*/	
	pid_init(&M3508_PID_Serve_Downpos[0], Position_Mode, 0.0100f , 0.002f, 0.001f, 5000, 6000);
	pid_init(&M3508_PID_Serve_Downpos[1], Position_Mode, 0.0100f , 0.002f, 0.001f, 5000, 6000);
	pid_init(&M3508_PID_Serve_Downpos[2], Position_Mode, 0.0100f , 0.002f, 0.001f, 5000, 6000);
	pid_init(&M3508_PID_Serve_Downpos[3], Position_Mode, 0.0100f , 0.002f, 0.001f, 5000, 6000);
}

/**************************************************************
	@brief:  �����۽�������Ŀ��λ���趨
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
	@brief:  �����۽�������Ŀ��λ���趨
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
	@brief:  �����۽�������Ŀ��λ���趨
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

#include "CushionBall_Task.h"
/*************************ȫ�ֱ���*************************/
extern Task_Flag task_flag;
float Scope_Print0,Scope_Print1,Scope_Print2,Scope_Print3;

float Vision_KP[3] = {0};
uint8_t Cushion_To_vision[4] = {"dia"};
float err;


int32_t Err_uppos[4]={0};
static int32_t up_tgpos = 22500;
static int32_t down_tgpos = 18000;
float vs_change_pos[4];
/*************************ȫ�ֺ���*************************/
//	���ϵĽǶȣ����µĽǶ�
// 	GO��������Ƕ�  10.71��00
void Arm_Get_Basepos(void);//��е�۳�ʼλ��ֵ��ȡ
void Arm_PID_Init	(void);//PID��ʼ��


void Get_Vision_Err(float Alpha);

/*�����Ӿ����ݵı仯PID����*/
//void Get_Vision_KP(float X_axis,float Y_axis);
//void Get_Mechanical_Error_KP(void);
void Vision_Poset(float X_axis,float Y_axis);
void Get_KP_Change(float distance);

void Get_Errpos_Judge(void);//�½��ж�
void Delta_UPSet_tgpos(void);//����Ŀ��λ���趨
void Delta_DownSet_tgpos(void);//�½�Ŀ��λ���趨
void Delta_frist_hit(void);//����һ�λ���߶���֮����������0��
/*Delta��е��PID����*/
void Delta_PID_POScalc(DjiMotor_State * ptr, pid_t * pid);
void Delta_PID_SPDcalc(DjiMotor_State * ptr, pid_t * pidpos, pid_t * pidspd);

void Get_Scope_Print(void);//ʾ��������
/*************************End*************************/

float cushion_time, cushion_last_time;

/*���ŵ��Ϊ���ٵ������������ͬ���������ĸ����*/
void CushionBall_Task(void const * argument)
{
	Arm_Get_Basepos();
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();//��ȡʱ�ӽ���
	for(;;)
	{
		cushion_time = HAL_GetTick() - cushion_last_time;
		cushion_last_time = HAL_GetTick();
		/*PID������ʼ��*/	
		Arm_PID_Init();
		Vision_Poset(Camera_Coordinate.X_axis,Camera_Coordinate.Y_axis);
		Get_Errpos_Judge();//�жϱ�־λ�������˶����������˶�
				/*�߼��ж�*/
				switch(task_flag.cushionball_flag)
				{
				/*���Ӿ����͵�����Ϣ*/
				case (Cushion_To_Vision):
				{
					HAL_UART_Transmit(&huart2,Cushion_To_vision,3,HAL_MAX_DELAY);
					HAL_Delay(800);
					task_flag.cushionball_flag = firstball;
				}
				/*���һ����*/
				case (firstball):
				{
					Delta_frist_hit();//������һ�ε���ĸ߶�ֵ
					Delta_PID_POScalc(M3508_State_Data, M3508_PID_Cushion_UPpos);
					Delta_PID_SPDcalc(M3508_State_Data, M3508_PID_Cushion_UPpos, M3508_PID_spd);
					
				} break;
				/*������������*/
				case (up_action):
				{
					Delta_UPSet_tgpos();//�������ϵ����λ��ֵ
					Delta_PID_POScalc(M3508_State_Data, M3508_PID_UPpos);
					Delta_PID_SPDcalc(M3508_State_Data, M3508_PID_UPpos, M3508_PID_spd);
				} break;
				/*�����½�����*/
				case (down_action):
				{
					/*�����س�λ��ֵ*/
					Delta_DownSet_tgpos();
					Delta_PID_POScalc(M3508_State_Data, M3508_PID_Downpos);
					Delta_PID_SPDcalc(M3508_State_Data, M3508_PID_Downpos, M3508_PID_spd);
					/*��һ�λس�֮��򿪹�翪��*/
					Light_On = 1;
				} break;
				default: break;
				}

		/*	�������� */
//		M3508_CAN_cmd(0x200, M3508_PID_spd[0].out, M3508_PID_spd[1].out, M3508_PID_spd[2].out, M3508_PID_spd[3].out);
		/*	ʾ������ӡ */
		Get_Scope_Print();
//		/*	����紫����һ������֪ͨ*/
//		vTaskNotifyGiveFromISR(LightRemote_Handle, &pxHigherPriorityTaskWoken);
		/*	�������������뺯������ */
		osDelayUntil(&xLastWakeTime, 5);//������ʱ
	}
}
/**************************************************************
	@brief:  �����ʼλ��ֵ��ȡ
	@param:		
	@retval: 		
	@supplement: 
**************************************************************/
void Arm_Get_Basepos(void)
{
	Get_Basepos(&M3508_State_Data[0]); 
	Get_Basepos(&M3508_State_Data[1]);
	Get_Basepos(&M3508_State_Data[2]);
	Get_Basepos(&M3508_State_Data[3]);
}
/**************************************************************
	@brief:  PID������ʼ��
	@param:		
	@retval: 		
	@supplement: 
**************************************************************/
void Arm_PID_Init	(void)
{		
	//�������λ��PID��ʼ��������ֵ									
	//�����˶�	������ʼ��											KP ��KI��KD,�����޷�,����޷�
	/*��һ����PID��ʼ������������΢��һЩ*/
	pid_init(&M3508_PID_Cushion_UPpos[0], Position_Mode, 0.50000f - 0.1f, 0.0f, 0.0f, 5000, 12000);
	pid_init(&M3508_PID_Cushion_UPpos[1], Position_Mode, 0.59000f - 0.1f, 0.0f, 0.0f, 5000, 12000);
	pid_init(&M3508_PID_Cushion_UPpos[2], Position_Mode, 0.50000f - 0.1f, 0.0f, 0.0f, 5000, 12000);
	pid_init(&M3508_PID_Cushion_UPpos[3], Position_Mode, 0.45000f - 0.1f, 0.0f, 0.0f, 5000, 12000);
	
  //���������˶�������ʼ��
	pid_init(&M3508_PID_UPpos[0], Position_Mode, 0.41500f + 0.210f, 0.0f, 0.0f, 0, 8000);
	pid_init(&M3508_PID_UPpos[1], Position_Mode, 0.70500f + 0.205f, 0.0f, 0.0f, 0, 8000);
	pid_init(&M3508_PID_UPpos[2], Position_Mode, 0.54000f + 0.310f, 0.0f, 0.0f, 0, 8000);
	pid_init(&M3508_PID_UPpos[3], Position_Mode, 0.60500f + 0.195f, 0.0f, 0.0f, 0, 8000);
	/*�ٶȻ� KD = 2.0f, KI = 0 ,KD = 0;*/
	pid_init(&M3508_PID_spd[0], Position_Mode, 2.000f, 0.00001f, 0.00f, 5000, 16000);
	pid_init(&M3508_PID_spd[1], Position_Mode, 2.003f, 0.00001f, 0.00f, 5000, 16000);
	pid_init(&M3508_PID_spd[2], Position_Mode, 2.003f, 0.00001f, 0.00f, 5000, 16000);
	pid_init(&M3508_PID_spd[3], Position_Mode, 2.003f, 0.00001f, 0.00f, 5000, 16000);
	/*���»س�  ������ʼ��*/ 
	pid_init(&M3508_PID_Downpos[0], Position_Mode, 0.300f, 0.0000f, 0.00f, 5000, 10000);
	pid_init(&M3508_PID_Downpos[1], Position_Mode, 0.290f, 0.0000f, 0.00f, 5000, 10000);
	pid_init(&M3508_PID_Downpos[2], Position_Mode, 0.300f, 0.0000f, 0.00f, 5000, 10000);
	pid_init(&M3508_PID_Downpos[3], Position_Mode, 0.310f, 0.0000f, 0.00f, 5000, 10000);
	/*����ģʽ�£�����λ�û���ʼ��*/ 
	pid_init(&M3508_PID_Hit_UPpos[0], Position_Mode, 0.9f, 0.0000f, 0.00f, 5000, 16000);
	pid_init(&M3508_PID_Hit_UPpos[1], Position_Mode, 0.9f, 0.0000f, 0.00f, 5000, 16000);
	pid_init(&M3508_PID_Hit_UPpos[2], Position_Mode, 0.9f, 0.0000f, 0.00f, 5000, 16000);
	pid_init(&M3508_PID_Hit_UPpos[3], Position_Mode, 0.9f, 0.0000f, 0.00f, 5000, 16000);
	/*����ģʽ�£��½�λ�û���ʼ��*/ 
	pid_init(&M3508_PID_Hit_Downpos[0], Position_Mode, 0.06f, 0.0000f, 0.00f, 5000, 10000);
	pid_init(&M3508_PID_Hit_Downpos[1], Position_Mode, 0.06f, 0.0000f, 0.00f, 5000, 10000);
	pid_init(&M3508_PID_Hit_Downpos[2], Position_Mode, 0.06f, 0.0000f, 0.00f, 5000, 10000);
	pid_init(&M3508_PID_Hit_Downpos[3], Position_Mode, 0.06f, 0.0000f, 0.00f, 5000, 10000);
}
/**************************************************************
	@brief:  ͨ���Ӿ��������ݣ�����X��������KPֵ
	@param:		
	@retval: 		
	@supplement: 
**************************************************************/
void Vision_Poset(float X_axis,float Y_axis)
{
	float Alpha, Length, Dx_Angle, Dy_Angle;
	float limit_length = 15000;//���ת���������ֵ 
	Alpha = atan2(Y_axis, X_axis);
	Alpha = Alpha + pi / 4;//˳ʱ����ת����ϵ
	Length = sqrt(X_axis * X_axis + Y_axis * Y_axis);
	/*Dx_Angle �� Dy_Angle����ֱ�Ӹ����ת������ֵ*/
	Dx_Angle = limit_length * Length * cos(Alpha) / 120;
	Dy_Angle = limit_length * Length * sin(Alpha) / 120;
	if((Length != 0) && (40 <= Length <= 130))
	{
		/*���ݳ��� length �ı�KPֵ*/
		Get_KP_Change(Length);
	//X����Ƕȸ�ֵ
	  if(Dx_Angle > 0)
	  {
		  vs_change_pos[1] = 0;
		  vs_change_pos[2] = Dx_Angle;

	  }
	  else if(Dx_Angle < 0)
	  {
		  vs_change_pos[1] = -Dx_Angle;//��Ϊ������ʹ�÷���Ƕ�ƫ��
		  vs_change_pos[2] = 0;
	  }
	  else
	  {
	  	vs_change_pos[1] = 0;
		  vs_change_pos[2] = 0;
	  }
	  //Y����Ƕȸ�ֵ
	  if(Dy_Angle > 0)
	  {
	  	vs_change_pos[0] = Dy_Angle;//����Ӧ��Ϊ Dy_Angle
		  vs_change_pos[3] = 0;
	  }
	  else if(Dy_Angle < 0)
	  {
		  vs_change_pos[0] = 0;
		  vs_change_pos[3] = -Dy_Angle;//��Ϊ������ʹ�÷���Ƕ�ƫ��
	  }
	  else
	  {
	  	vs_change_pos[0] = 0;
		  vs_change_pos[3] = 0;
	  }
  }

	else 
	{
		vs_change_pos[0] = 0;
		vs_change_pos[1] = 0;
		vs_change_pos[2] = 0;
		vs_change_pos[3] = 0;
	}

}

/**************************************************************
	@brief:  �ı���KPֵ������һ��KPֵ̫���ʹ�����߶�̫��
	@param:		
	@retval: 		
	@supplement: 
**************************************************************/
void Get_KP_Change(float distance)
{
	if( distance <= 40 )
	{
		/*1*/
		M3508_PID_UPpos[0].kp = M3508_PID_UPpos[0].kp ;
		M3508_PID_UPpos[1].kp = M3508_PID_UPpos[1].kp ;
		M3508_PID_UPpos[2].kp = M3508_PID_UPpos[2].kp ;
		M3508_PID_UPpos[3].kp = M3508_PID_UPpos[3].kp ;
	}
	else if ( 40 < distance <= 60)
	{
		/*30*/
		M3508_PID_UPpos[0].kp = M3508_PID_UPpos[0].kp * (0.4f + 0.065f * distance / 60.0f);
		M3508_PID_UPpos[1].kp = M3508_PID_UPpos[1].kp * (0.4f + 0.065f * distance / 60.0f);
		M3508_PID_UPpos[2].kp = M3508_PID_UPpos[2].kp * (0.4f + 0.065f * distance / 60.0f);
		M3508_PID_UPpos[3].kp = M3508_PID_UPpos[3].kp * (0.4f + 0.065f * distance / 60.0f);
	}
	else if ( 60 < distance <= 80)
	{
		/*30*/
		M3508_PID_UPpos[0].kp = M3508_PID_UPpos[0].kp * (0.2f + 0.025f * distance / 80.0f);
		M3508_PID_UPpos[1].kp = M3508_PID_UPpos[1].kp * (0.2f + 0.025f * distance / 80.0f);
		M3508_PID_UPpos[2].kp = M3508_PID_UPpos[2].kp * (0.2f + 0.025f * distance / 80.0f);
		M3508_PID_UPpos[3].kp = M3508_PID_UPpos[3].kp * (0.2f + 0.025f * distance / 80.0f);
	}
	else if ( 80 < distance <= 100 )
	{
		/*50*/
//		M3508_PID_UPpos[0].kp = M3508_PID_UPpos[0].kp / distance * 110.0f;
//		M3508_PID_UPpos[1].kp = M3508_PID_UPpos[1].kp / distance * 110.0f;
//		M3508_PID_UPpos[2].kp = M3508_PID_UPpos[2].kp / distance * 110.0f;
//		M3508_PID_UPpos[3].kp = M3508_PID_UPpos[3].kp / distance * 110.0f;
		
		M3508_PID_UPpos[0].kp = M3508_PID_UPpos[0].kp * (0.1f + 0.0025f * distance / 100.0f);
		M3508_PID_UPpos[1].kp = M3508_PID_UPpos[1].kp * (0.1f + 0.0025f * distance / 100.0f);
		M3508_PID_UPpos[2].kp = M3508_PID_UPpos[2].kp * (0.1f + 0.0025f * distance / 100.0f);
		M3508_PID_UPpos[3].kp = M3508_PID_UPpos[3].kp * (0.1f + 0.0025f * distance / 100.0f);
	}
	else if ( 100 < distance <= 120 )
	{
		M3508_PID_UPpos[0].kp = M3508_PID_UPpos[0].kp * (0.1f + 0.0025f * distance / 120.0f);
		M3508_PID_UPpos[1].kp = M3508_PID_UPpos[1].kp * (0.1f + 0.0025f * distance / 120.0f);
		M3508_PID_UPpos[2].kp = M3508_PID_UPpos[2].kp * (0.1f + 0.0025f * distance / 120.0f);
		M3508_PID_UPpos[3].kp = M3508_PID_UPpos[3].kp * (0.1f + 0.0025f * distance / 120.0f);
	}
	else if ( 120 < distance < 130 )
	{
		/*100*/
		M3508_PID_UPpos[0].kp = M3508_PID_UPpos[0].kp * (0.1f + 0.01f * distance / 130.0f);
		M3508_PID_UPpos[1].kp = M3508_PID_UPpos[1].kp * (0.1f + 0.01f * distance / 130.0f);
		M3508_PID_UPpos[2].kp = M3508_PID_UPpos[2].kp * (0.1f + 0.01f * distance / 130.0f);
		M3508_PID_UPpos[3].kp = M3508_PID_UPpos[3].kp * (0.1f + 0.01f * distance / 130.0f);
	}
	else if ( 130 < distance < 150 )
	{
		/*100*/
		M3508_PID_UPpos[0].kp = M3508_PID_UPpos[0].kp * (0.001f + 0.01f * distance / 150.0f);
		M3508_PID_UPpos[1].kp = M3508_PID_UPpos[1].kp * (0.001f + 0.01f * distance / 150.0f);
		M3508_PID_UPpos[2].kp = M3508_PID_UPpos[2].kp * (0.001f + 0.01f * distance / 150.0f);
		M3508_PID_UPpos[3].kp = M3508_PID_UPpos[3].kp * (0.001f + 0.01f * distance / 150.0f);
	}
}

/**************************************************************

	@brief:  ������ֵ���ж���һ���������˶���������1
	@param:		
	@retval: 		
	@supplement: 
**************************************************************/
void Get_Errpos_Judge(void)
{

	/*������ֵ���趨�˶�����*/
	Err_uppos[0] = up_tgpos + vs_change_pos[0] - 3500 ;
	Err_uppos[1] = up_tgpos + vs_change_pos[1] - 3500 ;
	Err_uppos[2] = up_tgpos + vs_change_pos[2] - 3500 ;
	Err_uppos[3] = up_tgpos + vs_change_pos[3] - 3500 ;
	/*ʵ��ֵ��Ŀ��ֵ�����жϣ��ж��˶�����*/
	if((M3508_State_Data[0].sumpos > Err_uppos[0]) && (M3508_State_Data[1].sumpos > Err_uppos[1]) && (M3508_State_Data[2].sumpos > Err_uppos[2])&&(M3508_State_Data[3].sumpos > Err_uppos[3])&&(task_flag.hitball_flag == 0))
	{
		task_flag.cushionball_flag = down_action;
	}
}
/**************************************************************

	@brief:  Delta��һ�λ���
	@param:		
	@retval: 		
	@supplement: �����ĽǶ� �� ת�ӽǶȻ���
**************************************************************/
void  Delta_frist_hit(void)
{
	M3508_State_Data[0].targetpos = 32000;
	M3508_State_Data[1].targetpos = 32000;
	M3508_State_Data[2].targetpos = 32000;
	M3508_State_Data[3].targetpos = 32000;
}
/**************************************************************

	@brief:  Delta�����˶��Ƕ�
	@param:		
	@retval: 		

	@supplement: �����ĽǶ� �� ת�ӽǶȻ���
**************************************************************/
void Delta_UPSet_tgpos(void)
{
	M3508_State_Data[0].targetpos = up_tgpos + vs_change_pos[0];
	M3508_State_Data[1].targetpos = up_tgpos + vs_change_pos[1];
	M3508_State_Data[2].targetpos = up_tgpos + vs_change_pos[2];
	M3508_State_Data[3].targetpos = up_tgpos + vs_change_pos[3];
}
/**************************************************************

	@brief:  Delta�����˶��Ƕ�
	@param:		
	@retval: 		
	@supplement: 
**************************************************************/
void Delta_DownSet_tgpos(void)
{	
	M3508_State_Data[0].targetpos =	down_tgpos;
	M3508_State_Data[1].targetpos = down_tgpos;
	M3508_State_Data[2].targetpos = down_tgpos;
	M3508_State_Data[3].targetpos = down_tgpos;
}
/**************************************************************
	@brief:  Delta�˶� 	λ�û�PID����
	@param:		
	@retval: 
	@supplement: 
							1. DjiMotor_State * ptr  ������һ����Ϊptr�Ľṹ��ָ�룬����ָ��һ��DjiMotor_State���͵ı������ڴ�λ�� 
								 ���� DjiMotor_State M3508_State_Data[3];
							2. pid_t * pid  ������һ����Ϊpid�Ľṹ��ָ��,����ָ��һ�� pid_t ���͵ı������ڴ�λ��
								 pid_t M3508_PID_UPpos[3];
***************************************************************/
void Delta_PID_POScalc(DjiMotor_State * ptr, pid_t * pid)
{
	pid_calc(ptr[0].targetpos, ptr[0].sumpos, &pid[0]);
	pid_calc(ptr[1].targetpos, ptr[1].sumpos, &pid[1]);
	pid_calc(ptr[2].targetpos, ptr[2].sumpos, &pid[2]);
	pid_calc(ptr[3].targetpos, ptr[3].sumpos, &pid[3]);
}
/**************************************************************
	@brief:  Delta�˶�	�ٶȻ�PID����
	@param:		
	@retval: 		
	@supplement: 
**************************************************************/
void Delta_PID_SPDcalc(DjiMotor_State * ptr, pid_t * pidpos, pid_t * pidspd)
{
	pid_calc(pidpos[0].out, ptr[0].spd, &pidspd[0]);
	pid_calc(pidpos[1].out, ptr[1].spd, &pidspd[1]);
	pid_calc(pidpos[2].out, ptr[2].spd, &pidspd[2]);
	pid_calc(pidpos[3].out, ptr[3].spd, &pidspd[3]);
}
/**************************************************************
	@brief: //J��Scopeʾ�����۲⣬���ֲ�������ֵ��ȫ�ֱ��������ڹ۲�
	@param:		
	@retval: 		
	@supplement: 
**************************************************************/
void Get_Scope_Print(void)
{
	err = M3508_State_Data[0].targetpos - M3508_State_Data[0].sumpos;
	Scope_Print0 = M3508_State_Data[0].sumpos;
	Scope_Print1 = M3508_State_Data[1].sumpos;
	Scope_Print2 = M3508_State_Data[2].sumpos;
	Scope_Print3 = M3508_State_Data[3].sumpos;
}
///**************************************************************
//	@brief: //�ֶ�����
//	@param:		
//	@retval: 		
//	@supplement: 
//**************************************************************/
//void Get_Vision_Err(float Alpha)
//{
//	/*�Ƕ�ֵת��Ϊ������*/
//	Camera_Coordinate.X_axis = L * cos(Alpha* pi / 180);
//	Camera_Coordinate.Y_axis = L * sin(Alpha* pi / 180);
//}

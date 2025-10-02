#include "ChassisDrive_Task.h"
/*****************************�궨��*****************************/
#define Max_Current 7500
/*****************************ȫ�ֱ���*****************************/
float Machine_Beta_Basepos, Machine_Beta_Targetpos;
uint8_t Chassis_Mode;
int Beta_Lock;
//int16_t Wheel_Current[3];
/*****************************ȫ�ֺ���*****************************/
extern float Kalman_X_axis;
extern float Kalman_Y_axis;

void Wheel_Spdcalc(void);
void World_PID_Poscalc(float tgx,  float tgy, float nowx, float nowy);
void Machine_Beta_PID_Poscalc(float tg_yaw,  float now_yaw);
void Chassis_PID_Init(void);
void Chassis_Get_Basepos(void);
void Dead_Zone_Set(float nowx, float nowy);
/*****************************End*****************************/

int32_t now_time[2], last_time;

void ChassisDrive_Task(void const * argument)
{
	/********************���̵��PID��ʼ��********************/
	
	/*�궨���̵����ʼλ��,�Լ����̵ĳ�ʼƫ����*/
	Chassis_Get_Basepos();
	
	/*����������һ������xLastWakeTime������������TickType_t
	TickType_t��һ����FreeRTOS������������ͣ�������ʾʱ�ӽ��ġ�*/
	TickType_t xLastWakeTime;
	/*����xTaskGetTickCount()��������ȡ��ǰ��ʱ�ӽ��ļ������������ֵ��������xLastWakeTime*/
	xLastWakeTime = xTaskGetTickCount();//��ȡʱ�ӽ���
	
	for(;;)
	{
		//�����������ʱ��
		ChassisDriveTask_Moment = Get_Systerm_Time();
		//���ݵ������ָ�����PID��ʼ������
		Chassis_PID_Init();
		//�����˶�ģʽѡ��
		switch(Chassis_Mode)
		{
		/****************�Զ�ģʽ****************/
		  case Automatic_Mode://�Ӿ���������
			{
				/*�����Ӿ����ݼ����ʵ�ʵĺ�����Լ�����������ٶ�*/
				World_PID_Poscalc(Kalman_X_axis, Kalman_Y_axis, 0, 0);//��������λ��������Ӿ�����
				
				/*��������ʱ��Ŀ���ƫ����Ϊ���̳�ʼ��ʱ��ƫ����*/
				/*�������ƫ�������ݹߵ�������ƫ���Ǳ仯������������Ľ��ٶ�*/
				Machine_Beta_PID_Poscalc(Machine_Beta_Targetpos, Chassis_Machine.Beta);
			} break;
		/****************�ֶ�ģʽ****************/
			case Operation_Mode://App����
			{
			  if(Beta_Lock == 1)
				{
					Machine_Beta_PID_Poscalc(Machine_Beta_Targetpos, Chassis_Machine.Beta);
				}
			} break;
		  /****************�ƶ�ģʽ****************/
			case Retardation_Mode://��ֵ��Vision_Task��
			{
				World_PID_Poscalc(0.0f, -6.5f, 0, 0);//ɲ���ƶ�	
				Machine_Beta_PID_Poscalc(Machine_Beta_Targetpos, Chassis_Machine.Beta);
			} break;
			default: break; 
		}
		//����������ϵת��Ϊ�������ϵ���õ�Ŀ��ת��
		Chassis_Drive(&Chassis_World, &Chassis_Machine, &Chassis_Wheel);
		//�ٶȻ�PID������˶�ѧ���У��                                                                                                                                       
		Wheel_Spdcalc();
		//��������
		M3508_CAN_cmd(0x200, M3508_PIDspd[0].out, M3508_PIDspd[1].out, M3508_PIDspd[2].out);
		osDelayUntil(&xLastWakeTime, 4);//������ʱ
	}
}

/**************************************************************
	@brief:  ���̵��PID��ʼ��
	@param:		
	@retval: 		
	@supplement: 
**************************************************************/
void Chassis_PID_Init(void)
{
	if(Hit_Enable)//����ģʽ����
	{
	//����λ��PID��ʼ�������������׼,X��,Y�ᣩ
	  pid_init(&World_Chassis_X_PIDpos, Position_Mode, 9.5f, 0, 0, 0, 2000);
	  pid_init(&World_Chassis_Y_PIDpos, Position_Mode, 9.5f, 0, 0, 0, 2000);
	//��������PID��ʼ������������ϵƫ���ǣ�
	  pid_init(&Machine_Beta_PIDpos, Position_Mode, 0.2f, 0, 1.0f, 0, 1300);
	//���̵��PID��ʼ���������׼ ��
	  pid_init(&M3508_PIDspd[0], Position_Mode, 10.0f, 0, 0, 0, Max_Current);//Max_Current��Ϊ8000
	  pid_init(&M3508_PIDspd[1], Position_Mode, 10.0f, 0, 0, 0, Max_Current);
	  pid_init(&M3508_PIDspd[2], Position_Mode, 10.0f, 0, 0, 0, Max_Current);
	}
	if(Cushion_Enable)//����ģʽ����
	{
	//����λ��PID��ʼ�������������׼,X��,Y�ᣩ
	  pid_init(&World_Chassis_X_PIDpos, Position_Mode, 6.2f+0.5f, 0, 0, 0, 2300);
	  pid_init(&World_Chassis_Y_PIDpos, Position_Mode, 6.2f+0.5f, 0, 0, 0, 2300);
	//��������PID��ʼ������������ϵƫ���ǣ�
	  pid_init(&Machine_Beta_PIDpos, Position_Mode, 0.2f, 0, 1.0f, 0, 1300);
	//���̵��PID��ʼ���������׼ ��
	  pid_init(&M3508_PIDspd[0], Position_Mode, 7.0f, 0, 0, 0, 6700);//Max_Current��Ϊ10000
	  pid_init(&M3508_PIDspd[1], Position_Mode, 7.0f, 0, 0, 0, 6700);
	  pid_init(&M3508_PIDspd[2], Position_Mode, 7.0f, 0, 0, 0, 6700);
	}
}
/**************************************************************
	@brief:  ���̵����ʼλ��ֵ��ȡ
	@param:		
	@retval: 		
	@supplement: 
**************************************************************/
void Chassis_Get_Basepos(void)
{
	Get_Basepos(&M3508_State_Data[0]);
	Get_Basepos(&M3508_State_Data[1]);
	Get_Basepos(&M3508_State_Data[2]);
	/*�ϵ��ʼ��ʱ ����δ��Machine_Beta_Basepos��ֵ
		���� Machine_Beta_Basepos == 0 ����*/
	while(Machine_Beta_Basepos == 0)
	{
		Machine_Beta_Basepos = Chassis_Machine.Beta;
	}
	/*ȷ�����̳�ʼ��ƫ����*/
	Machine_Beta_Targetpos = Machine_Beta_Basepos;
}
/**************************************************************
	@brief:  ƽ��λ�õ��ٶȵļ���
	@param:		
	@retval: 		
	@supplement: 1.���ڵ��̵�λ�ÿ��� 
               2.����ϵΪ��������ϵ
               3.����Ϊ�Ӿ���λ��ƫ��ֵ
               4.�ڵ���ģʽ�У���������ϵ���������ϵ�غ�,������˶�
							 5.�õ���������ϵ�ĺ�����Լ�ƽ���ٶ�
**************************************************************/
void World_PID_Poscalc(float tgx,  float tgy, float nowx, float nowy)
{
	static float x_out = 0, y_out = 0;
//	float k = 0.1f;
	
	/*X������ λ������Ŀ��ֵ��ʵ��ֵ�����õĽṹ��*/
	pid_calc(tgx, nowx, &World_Chassis_X_PIDpos);
	x_out = World_Chassis_X_PIDpos.out;
	/*Y������ λ������Ŀ��ֵ��ʵ��ֵ�����õĽṹ��*/
	pid_calc(tgy, nowy, &World_Chassis_Y_PIDpos);
	y_out = World_Chassis_Y_PIDpos.out;
	/*λ�û��������Ϊ�ٶȻ������룬�õ���������Ŀ�����ٶ�*/
	Chassis_World.Vel = sqrt(x_out * x_out + y_out * y_out);
	
	/*�õ������ĺ����*/
	Chassis_World.Alpha = RAD2ANGLE(atan2((tgy - nowy), (tgx - nowx)));
	/*�Ƕ��ж�Ҫע���Ӿ����������Ƿ�����������������ƥ��*/
	/*��Ҫ����PID�����ֵ���нǶ����㣬���ܽ��������*/
	/*atan2(y,x) = arctan(y/x), �ҿ����˽Ƕȵ�������������*/
}
/**************************************************************
	@brief:  ��������λ�õ��ٶȵļ��㣬
	@param:		
	@retval: 		
	@supplement: 1.���ڵ��̵�ƫ��λ�ñջ�
               2.����ϵΪ��������ϵ
               3.����ΪĿ��ƫ���Ƕ�ֵ
               4.����ģʽ�£�����ʱ���ú���Ӧ��������
               5.��Ҫ���нǶȱջ�����ƫ��������ƫ���Ƕȸ���ʱ
**************************************************************/
void Machine_Beta_PID_Poscalc(float tg_yaw,  float now_yaw)
{
	/*�����������ٶȼ���*/
	pid_calc(tg_yaw, now_yaw, &Machine_Beta_PIDpos);
	Chassis_Machine.velW = Machine_Beta_PIDpos.out;
}
/**************************************************************
	@brief:  ���̸���ת��PID����
	@param:		
	@retval: 		
	@supplement: 1.���ڵ������ӵ��ٶȿ���
				 2.����ϵΪ��������ϵ
**************************************************************/
int Max_Id = 0;
void Wheel_Spdcalc(void)
{
	float wheel_tgspd[3];
	/*************PID����*************/
	pid_calc(Chassis_Wheel.wheel1, M3508_State_Data[0].spd, &M3508_PIDspd[0]);
	pid_calc(Chassis_Wheel.wheel2, M3508_State_Data[1].spd, &M3508_PIDspd[1]);
	pid_calc(Chassis_Wheel.wheel3, M3508_State_Data[2].spd, &M3508_PIDspd[2]);
	
	/*************���������ֵĿ���ٶ�*************/
	/*fabs(float/double x),���㸡���� x �ľ���ֵ*/
	wheel_tgspd[0] = fabs(Chassis_Wheel.wheel1);
	wheel_tgspd[1] = fabs(Chassis_Wheel.wheel2);
	wheel_tgspd[2] = fabs(Chassis_Wheel.wheel3);
	
	Max_Id = Bubble_Sort(wheel_tgspd, 3);//�õ�������ֵID

	/*************�����������������ٶ�,ʹ����϶���ѧ��ϵ������PID����޷����±���
	�������Ӹ���PID������ͬ����������Ŀ��ֵ��ͬ��errֵ��ͬ������ʱ�������ֵ��һ������ȫ�����˶�ѧ��ϵ*************/
	
	if(M3508_PIDspd[Max_Id].out != 0)
	{
		switch(Max_Id)
		{
			case 0:
			{
				M3508_PIDspd[1].out = (Chassis_Wheel.wheel2 / Chassis_Wheel.wheel1) * M3508_PIDspd[0].out;
				M3508_PIDspd[2].out = (Chassis_Wheel.wheel3 / Chassis_Wheel.wheel1) * M3508_PIDspd[0].out;
			} break;
			case 1:
			{
				M3508_PIDspd[0].out = (Chassis_Wheel.wheel1 / Chassis_Wheel.wheel2) * M3508_PIDspd[1].out;
				M3508_PIDspd[2].out = (Chassis_Wheel.wheel3 / Chassis_Wheel.wheel2) * M3508_PIDspd[1].out;
			} break;
			case 2:
			{
				M3508_PIDspd[0].out = (Chassis_Wheel.wheel1 / Chassis_Wheel.wheel3) * M3508_PIDspd[2].out;
				M3508_PIDspd[1].out = (Chassis_Wheel.wheel2 / Chassis_Wheel.wheel3) * M3508_PIDspd[2].out;
			} break;
			default: break;
		}
	}
//	// ��PID�������ָ����Ȩƽ��ƽ������
//  static float previous_ewma[3] = {0}; // ��ʼֵΪ0��Ҳ���Ը�����Ҫ��ʼ��
//  M3508_PIDspd[0].out = exponential_weighted_moving_average(M3508_PIDspd[0].out, previous_ewma[0]);
//  M3508_PIDspd[1].out = exponential_weighted_moving_average(M3508_PIDspd[1].out, previous_ewma[1]);
//  M3508_PIDspd[2].out = exponential_weighted_moving_average(M3508_PIDspd[2].out, previous_ewma[2]);
//  // ������ʷEWMAֵ
//  previous_ewma[0] = M3508_PIDspd[0].out;
//  previous_ewma[1] = M3508_PIDspd[1].out;
//  previous_ewma[2] = M3508_PIDspd[2].out;
	/*************������ֵĿ���ٶ�Ϊ0����������Ӧ��Ϊ0*************/
	if(M3508_PIDspd[Max_Id].out == 0)
	{
		M3508_PIDspd[0].out = 0;
		M3508_PIDspd[1].out = 0;
		M3508_PIDspd[2].out = 0;
	}
}
/**************************************************************
	@brief:  ���������趨
	@param:		
	@retval: 		
	@supplement: ������һ���Ӿ�������ֹͣ��������
**************************************************************/
void Dead_Zone_Set(float nowx, float nowy)
{
	float length;
	length = sqrt(nowx * nowx + nowy * nowy);
	if(Chassis_Mode == Automatic_Mode)
	{
		if(length < 10)
	  {
		  M3508_PIDspd[0].out = 0;
		  M3508_PIDspd[1].out = 0;
		  M3508_PIDspd[2].out = 0;
	  }
  }
}

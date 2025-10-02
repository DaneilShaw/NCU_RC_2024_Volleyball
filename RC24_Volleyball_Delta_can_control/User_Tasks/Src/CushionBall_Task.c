#include "CushionBall_Task.h"
/*************************全局变量*************************/
extern Task_Flag task_flag;
float Scope_Print0,Scope_Print1,Scope_Print2,Scope_Print3;

float Vision_KP[3] = {0};
uint8_t Cushion_To_vision[4] = {"dia"};
float err;


int32_t Err_uppos[4]={0};
static int32_t up_tgpos = 22500;
static int32_t down_tgpos = 18000;
float vs_change_pos[4];
/*************************全局函数*************************/
//	向上的角度，向下的角度
// 	GO电机俯仰角度  10.71°00
void Arm_Get_Basepos(void);//机械臂初始位置值获取
void Arm_PID_Init	(void);//PID初始化


void Get_Vision_Err(float Alpha);

/*基于视觉数据的变化PID参数*/
//void Get_Vision_KP(float X_axis,float Y_axis);
//void Get_Mechanical_Error_KP(void);
void Vision_Poset(float X_axis,float Y_axis);
void Get_KP_Change(float distance);

void Get_Errpos_Judge(void);//下降判断
void Delta_UPSet_tgpos(void);//上升目标位置设定
void Delta_DownSet_tgpos(void);//下降目标位置设定
void Delta_frist_hit(void);//将第一次击球高度与之后垫球的区分0开
/*Delta机械臂PID计算*/
void Delta_PID_POScalc(DjiMotor_State * ptr, pid_t * pid);
void Delta_PID_SPDcalc(DjiMotor_State * ptr, pid_t * pidpos, pid_t * pidspd);

void Get_Scope_Print(void);//示波器函数
/*************************End*************************/

float cushion_time, cushion_last_time;

/*三号电机为卡顿电机，尝试用相同参数驱动四个电机*/
void CushionBall_Task(void const * argument)
{
	Arm_Get_Basepos();
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();//获取时钟节拍
	for(;;)
	{
		cushion_time = HAL_GetTick() - cushion_last_time;
		cushion_last_time = HAL_GetTick();
		/*PID参数初始化*/	
		Arm_PID_Init();
		Vision_Poset(Camera_Coordinate.X_axis,Camera_Coordinate.Y_axis);
		Get_Errpos_Judge();//判断标志位，向下运动还是向下运动
				/*逻辑判断*/
				switch(task_flag.cushionball_flag)
				{
				/*给视觉发送垫球信息*/
				case (Cushion_To_Vision):
				{
					HAL_UART_Transmit(&huart2,Cushion_To_vision,3,HAL_MAX_DELAY);
					HAL_Delay(800);
					task_flag.cushionball_flag = firstball;
				}
				/*垫第一个球*/
				case (firstball):
				{
					Delta_frist_hit();//给定第一次垫球的高度值
					Delta_PID_POScalc(M3508_State_Data, M3508_PID_Cushion_UPpos);
					Delta_PID_SPDcalc(M3508_State_Data, M3508_PID_Cushion_UPpos, M3508_PID_spd);
					
				} break;
				/*后续上升动作*/
				case (up_action):
				{
					Delta_UPSet_tgpos();//给定向上垫球的位置值
					Delta_PID_POScalc(M3508_State_Data, M3508_PID_UPpos);
					Delta_PID_SPDcalc(M3508_State_Data, M3508_PID_UPpos, M3508_PID_spd);
				} break;
				/*后续下降动作*/
				case (down_action):
				{
					/*给定回程位置值*/
					Delta_DownSet_tgpos();
					Delta_PID_POScalc(M3508_State_Data, M3508_PID_Downpos);
					Delta_PID_SPDcalc(M3508_State_Data, M3508_PID_Downpos, M3508_PID_spd);
					/*第一次回程之后打开光电开关*/
					Light_On = 1;
				} break;
				default: break;
				}

		/*	电流发送 */
//		M3508_CAN_cmd(0x200, M3508_PID_spd[0].out, M3508_PID_spd[1].out, M3508_PID_spd[2].out, M3508_PID_spd[3].out);
		/*	示波器打印 */
		Get_Scope_Print();
//		/*	给光电传感器一个任务通知*/
//		vTaskNotifyGiveFromISR(LightRemote_Handle, &pxHigherPriorityTaskWoken);
		/*	任务阻塞，进入函数调度 */
		osDelayUntil(&xLastWakeTime, 5);//绝对延时
	}
}
/**************************************************************
	@brief:  电机初始位置值获取
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
	@brief:  PID参数初始化
	@param:		
	@retval: 		
	@supplement: 
**************************************************************/
void Arm_PID_Init	(void)
{		
	//三个电机位置PID初始化，并赋值									
	//向上运动	参数初始化											KP ，KI，KD,积分限幅,输出限幅
	/*第一个球PID初始化，将球垫的稍微高一些*/
	pid_init(&M3508_PID_Cushion_UPpos[0], Position_Mode, 0.50000f - 0.1f, 0.0f, 0.0f, 5000, 12000);
	pid_init(&M3508_PID_Cushion_UPpos[1], Position_Mode, 0.59000f - 0.1f, 0.0f, 0.0f, 5000, 12000);
	pid_init(&M3508_PID_Cushion_UPpos[2], Position_Mode, 0.50000f - 0.1f, 0.0f, 0.0f, 5000, 12000);
	pid_init(&M3508_PID_Cushion_UPpos[3], Position_Mode, 0.45000f - 0.1f, 0.0f, 0.0f, 5000, 12000);
	
  //垫球，向上运动参数初始化
	pid_init(&M3508_PID_UPpos[0], Position_Mode, 0.41500f + 0.210f, 0.0f, 0.0f, 0, 8000);
	pid_init(&M3508_PID_UPpos[1], Position_Mode, 0.70500f + 0.205f, 0.0f, 0.0f, 0, 8000);
	pid_init(&M3508_PID_UPpos[2], Position_Mode, 0.54000f + 0.310f, 0.0f, 0.0f, 0, 8000);
	pid_init(&M3508_PID_UPpos[3], Position_Mode, 0.60500f + 0.195f, 0.0f, 0.0f, 0, 8000);
	/*速度环 KD = 2.0f, KI = 0 ,KD = 0;*/
	pid_init(&M3508_PID_spd[0], Position_Mode, 2.000f, 0.00001f, 0.00f, 5000, 16000);
	pid_init(&M3508_PID_spd[1], Position_Mode, 2.003f, 0.00001f, 0.00f, 5000, 16000);
	pid_init(&M3508_PID_spd[2], Position_Mode, 2.003f, 0.00001f, 0.00f, 5000, 16000);
	pid_init(&M3508_PID_spd[3], Position_Mode, 2.003f, 0.00001f, 0.00f, 5000, 16000);
	/*向下回程  参数初始化*/ 
	pid_init(&M3508_PID_Downpos[0], Position_Mode, 0.300f, 0.0000f, 0.00f, 5000, 10000);
	pid_init(&M3508_PID_Downpos[1], Position_Mode, 0.290f, 0.0000f, 0.00f, 5000, 10000);
	pid_init(&M3508_PID_Downpos[2], Position_Mode, 0.300f, 0.0000f, 0.00f, 5000, 10000);
	pid_init(&M3508_PID_Downpos[3], Position_Mode, 0.310f, 0.0000f, 0.00f, 5000, 10000);
	/*发球模式下，上升位置环初始化*/ 
	pid_init(&M3508_PID_Hit_UPpos[0], Position_Mode, 0.9f, 0.0000f, 0.00f, 5000, 16000);
	pid_init(&M3508_PID_Hit_UPpos[1], Position_Mode, 0.9f, 0.0000f, 0.00f, 5000, 16000);
	pid_init(&M3508_PID_Hit_UPpos[2], Position_Mode, 0.9f, 0.0000f, 0.00f, 5000, 16000);
	pid_init(&M3508_PID_Hit_UPpos[3], Position_Mode, 0.9f, 0.0000f, 0.00f, 5000, 16000);
	/*发球模式下，下降位置环初始化*/ 
	pid_init(&M3508_PID_Hit_Downpos[0], Position_Mode, 0.06f, 0.0000f, 0.00f, 5000, 10000);
	pid_init(&M3508_PID_Hit_Downpos[1], Position_Mode, 0.06f, 0.0000f, 0.00f, 5000, 10000);
	pid_init(&M3508_PID_Hit_Downpos[2], Position_Mode, 0.06f, 0.0000f, 0.00f, 5000, 10000);
	pid_init(&M3508_PID_Hit_Downpos[3], Position_Mode, 0.06f, 0.0000f, 0.00f, 5000, 10000);
}
/**************************************************************
	@brief:  通过视觉反馈数据，调节X方向电机的KP值
	@param:		
	@retval: 		
	@supplement: 
**************************************************************/
void Vision_Poset(float X_axis,float Y_axis)
{
	float Alpha, Length, Dx_Angle, Dy_Angle;
	float limit_length = 15000;//电机转子最大增加值 
	Alpha = atan2(Y_axis, X_axis);
	Alpha = Alpha + pi / 4;//顺时针旋转坐标系
	Length = sqrt(X_axis * X_axis + Y_axis * Y_axis);
	/*Dx_Angle 和 Dy_Angle都是直接给电机转子增量值*/
	Dx_Angle = limit_length * Length * cos(Alpha) / 120;
	Dy_Angle = limit_length * Length * sin(Alpha) / 120;
	if((Length != 0) && (40 <= Length <= 130))
	{
		/*根据长度 length 改变KP值*/
		Get_KP_Change(Length);
	//X方向角度赋值
	  if(Dx_Angle > 0)
	  {
		  vs_change_pos[1] = 0;
		  vs_change_pos[2] = Dx_Angle;

	  }
	  else if(Dx_Angle < 0)
	  {
		  vs_change_pos[1] = -Dx_Angle;//改为正数，使该方向角度偏大
		  vs_change_pos[2] = 0;
	  }
	  else
	  {
	  	vs_change_pos[1] = 0;
		  vs_change_pos[2] = 0;
	  }
	  //Y方向角度赋值
	  if(Dy_Angle > 0)
	  {
	  	vs_change_pos[0] = Dy_Angle;//正常应该为 Dy_Angle
		  vs_change_pos[3] = 0;
	  }
	  else if(Dy_Angle < 0)
	  {
		  vs_change_pos[0] = 0;
		  vs_change_pos[3] = -Dy_Angle;//改为正数，使该方向角度偏大
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
	@brief:  改变电机KP值，避免一侧KP值太大而使球垫起高度太高
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

	@brief:  获得误差值并判断下一步是向上运动还是向下1
	@param:		
	@retval: 		
	@supplement: 
**************************************************************/
void Get_Errpos_Judge(void)
{

	/*获得误差值，设定运动区间*/
	Err_uppos[0] = up_tgpos + vs_change_pos[0] - 3500 ;
	Err_uppos[1] = up_tgpos + vs_change_pos[1] - 3500 ;
	Err_uppos[2] = up_tgpos + vs_change_pos[2] - 3500 ;
	Err_uppos[3] = up_tgpos + vs_change_pos[3] - 3500 ;
	/*实际值与目标值进行判断，判断运动方向*/
	if((M3508_State_Data[0].sumpos > Err_uppos[0]) && (M3508_State_Data[1].sumpos > Err_uppos[1]) && (M3508_State_Data[2].sumpos > Err_uppos[2])&&(M3508_State_Data[3].sumpos > Err_uppos[3])&&(task_flag.hitball_flag == 0))
	{
		task_flag.cushionball_flag = down_action;
	}
}
/**************************************************************

	@brief:  Delta第一次击球
	@param:		
	@retval: 		
	@supplement: 输出轴的角度 与 转子角度换算
**************************************************************/
void  Delta_frist_hit(void)
{
	M3508_State_Data[0].targetpos = 32000;
	M3508_State_Data[1].targetpos = 32000;
	M3508_State_Data[2].targetpos = 32000;
	M3508_State_Data[3].targetpos = 32000;
}
/**************************************************************

	@brief:  Delta向上运动角度
	@param:		
	@retval: 		

	@supplement: 输出轴的角度 与 转子角度换算
**************************************************************/
void Delta_UPSet_tgpos(void)
{
	M3508_State_Data[0].targetpos = up_tgpos + vs_change_pos[0];
	M3508_State_Data[1].targetpos = up_tgpos + vs_change_pos[1];
	M3508_State_Data[2].targetpos = up_tgpos + vs_change_pos[2];
	M3508_State_Data[3].targetpos = up_tgpos + vs_change_pos[3];
}
/**************************************************************

	@brief:  Delta向下运动角度
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
	@brief:  Delta运动 	位置环PID计算
	@param:		
	@retval: 
	@supplement: 
							1. DjiMotor_State * ptr  声明了一个名为ptr的结构体指针，可以指向一个DjiMotor_State类型的变量或内存位置 
								 例如 DjiMotor_State M3508_State_Data[3];
							2. pid_t * pid  声明了一个名为pid的结构体指针,可以指向一个 pid_t 类型的变量或内存位置
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
	@brief:  Delta运动	速度环PID计算
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
	@brief: //J—Scope示波器观测，将局部变量赋值给全局变量，易于观测
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
//	@brief: //手动调参
//	@param:		
//	@retval: 		
//	@supplement: 
//**************************************************************/
//void Get_Vision_Err(float Alpha)
//{
//	/*角度值转换为弧度制*/
//	Camera_Coordinate.X_axis = L * cos(Alpha* pi / 180);
//	Camera_Coordinate.Y_axis = L * sin(Alpha* pi / 180);
//}

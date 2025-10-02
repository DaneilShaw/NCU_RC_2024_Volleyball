#include "ChassisDrive_Task.h"
/*****************************宏定义*****************************/
#define Max_Current 7500
/*****************************全局变量*****************************/
float Machine_Beta_Basepos, Machine_Beta_Targetpos;
uint8_t Chassis_Mode;
int Beta_Lock;
//int16_t Wheel_Current[3];
/*****************************全局函数*****************************/
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
	/********************底盘电机PID初始化********************/
	
	/*标定底盘电机初始位置,以及底盘的初始偏航角*/
	Chassis_Get_Basepos();
	
	/*这里声明了一个变量xLastWakeTime，它的类型是TickType_t
	TickType_t是一个由FreeRTOS定义的数据类型，用来表示时钟节拍。*/
	TickType_t xLastWakeTime;
	/*调用xTaskGetTickCount()函数来获取当前的时钟节拍计数，并将这个值赋给变量xLastWakeTime*/
	xLastWakeTime = xTaskGetTickCount();//获取时钟节拍
	
	for(;;)
	{
		//更新任务调度时间
		ChassisDriveTask_Moment = Get_Systerm_Time();
		//根据垫球或发球指令，调整PID初始化参数
		Chassis_PID_Init();
		//底盘运动模式选择
		switch(Chassis_Mode)
		{
		/****************自动模式****************/
		  case Automatic_Mode://视觉数据驱动
			{
				/*根据视觉数据计算出实际的航向角以及整体机器的速度*/
				World_PID_Poscalc(Kalman_X_axis, Kalman_Y_axis, 0, 0);//世界坐标位置误差由视觉传入
				
				/*地盘自锁时，目标的偏航角为地盘初始化时的偏航角*/
				/*如果不锁偏航，根据惯导反馈的偏航角变化，计算出自旋的角速度*/
				Machine_Beta_PID_Poscalc(Machine_Beta_Targetpos, Chassis_Machine.Beta);
			} break;
		/****************手动模式****************/
			case Operation_Mode://App驱动
			{
			  if(Beta_Lock == 1)
				{
					Machine_Beta_PID_Poscalc(Machine_Beta_Targetpos, Chassis_Machine.Beta);
				}
			} break;
		  /****************制动模式****************/
			case Retardation_Mode://赋值在Vision_Task中
			{
				World_PID_Poscalc(0.0f, -6.5f, 0, 0);//刹车制动	
				Machine_Beta_PID_Poscalc(Machine_Beta_Targetpos, Chassis_Machine.Beta);
			} break;
			default: break; 
		}
		//将世界坐标系转化为电机坐标系，得到目标转速
		Chassis_Drive(&Chassis_World, &Chassis_Machine, &Chassis_Wheel);
		//速度环PID运算和运动学输出校正                                                                                                                                       
		Wheel_Spdcalc();
		//电流发送
		M3508_CAN_cmd(0x200, M3508_PIDspd[0].out, M3508_PIDspd[1].out, M3508_PIDspd[2].out);
		osDelayUntil(&xLastWakeTime, 4);//绝对延时
	}
}

/**************************************************************
	@brief:  底盘电机PID初始化
	@param:		
	@retval: 		
	@supplement: 
**************************************************************/
void Chassis_PID_Init(void)
{
	if(Hit_Enable)//发球模式代码
	{
	//底盘位置PID初始化（世界坐标基准,X轴,Y轴）
	  pid_init(&World_Chassis_X_PIDpos, Position_Mode, 9.5f, 0, 0, 0, 2000);
	  pid_init(&World_Chassis_Y_PIDpos, Position_Mode, 9.5f, 0, 0, 0, 2000);
	//底盘自旋PID初始化（机器坐标系偏航角）
	  pid_init(&Machine_Beta_PIDpos, Position_Mode, 0.2f, 0, 1.0f, 0, 1300);
	//底盘电机PID初始化（电机基准 ）
	  pid_init(&M3508_PIDspd[0], Position_Mode, 10.0f, 0, 0, 0, Max_Current);//Max_Current现为8000
	  pid_init(&M3508_PIDspd[1], Position_Mode, 10.0f, 0, 0, 0, Max_Current);
	  pid_init(&M3508_PIDspd[2], Position_Mode, 10.0f, 0, 0, 0, Max_Current);
	}
	if(Cushion_Enable)//垫球模式代码
	{
	//底盘位置PID初始化（世界坐标基准,X轴,Y轴）
	  pid_init(&World_Chassis_X_PIDpos, Position_Mode, 6.2f+0.5f, 0, 0, 0, 2300);
	  pid_init(&World_Chassis_Y_PIDpos, Position_Mode, 6.2f+0.5f, 0, 0, 0, 2300);
	//底盘自旋PID初始化（机器坐标系偏航角）
	  pid_init(&Machine_Beta_PIDpos, Position_Mode, 0.2f, 0, 1.0f, 0, 1300);
	//底盘电机PID初始化（电机基准 ）
	  pid_init(&M3508_PIDspd[0], Position_Mode, 7.0f, 0, 0, 0, 6700);//Max_Current现为10000
	  pid_init(&M3508_PIDspd[1], Position_Mode, 7.0f, 0, 0, 0, 6700);
	  pid_init(&M3508_PIDspd[2], Position_Mode, 7.0f, 0, 0, 0, 6700);
	}
}
/**************************************************************
	@brief:  底盘电机初始位置值获取
	@param:		
	@retval: 		
	@supplement: 
**************************************************************/
void Chassis_Get_Basepos(void)
{
	Get_Basepos(&M3508_State_Data[0]);
	Get_Basepos(&M3508_State_Data[1]);
	Get_Basepos(&M3508_State_Data[2]);
	/*上电初始化时 ，并未对Machine_Beta_Basepos赋值
		所以 Machine_Beta_Basepos == 0 成立*/
	while(Machine_Beta_Basepos == 0)
	{
		Machine_Beta_Basepos = Chassis_Machine.Beta;
	}
	/*确定底盘初始的偏航角*/
	Machine_Beta_Targetpos = Machine_Beta_Basepos;
}
/**************************************************************
	@brief:  平移位置到速度的计算
	@param:		
	@retval: 		
	@supplement: 1.用于底盘的位置控制 
               2.坐标系为世界坐标系
               3.输入为视觉的位置偏差值
               4.在垫球模式中，世界坐标系与机器坐标系重合,即相对运动
							 5.得到世界坐标系的航向角以及平动速度
**************************************************************/
void World_PID_Poscalc(float tgx,  float tgy, float nowx, float nowy)
{
	static float x_out = 0, y_out = 0;
//	float k = 0.1f;
	
	/*X方向上 位置数据目标值与实际值，作用的结构体*/
	pid_calc(tgx, nowx, &World_Chassis_X_PIDpos);
	x_out = World_Chassis_X_PIDpos.out;
	/*Y方向上 位置数据目标值与实际值，作用的结构体*/
	pid_calc(tgy, nowy, &World_Chassis_Y_PIDpos);
	y_out = World_Chassis_Y_PIDpos.out;
	/*位置环的输出作为速度环的输入，得到机器运行目标总速度*/
	Chassis_World.Vel = sqrt(x_out * x_out + y_out * y_out);
	
	/*得到机器的航向角*/
	Chassis_World.Alpha = RAD2ANGLE(atan2((tgy - nowy), (tgx - nowx)));
	/*角度判断要注意视觉给的数据是否与自身世界坐标轴匹配*/
	/*不要采用PID的输出值进行角度运算，可能将方向变歪*/
	/*atan2(y,x) = arctan(y/x), 且考虑了角度的正负进行运算*/
}
/**************************************************************
	@brief:  机器自旋位置到速度的计算，
	@param:		
	@retval: 		
	@supplement: 1.用于底盘的偏航位置闭环
               2.坐标系为机器坐标系
               3.输入为目标偏航角度值
               4.操作模式下，自旋时，该函数应不被调用
               5.需要进行角度闭环，如偏航自锁或偏航角度跟踪时
**************************************************************/
void Machine_Beta_PID_Poscalc(float tg_yaw,  float now_yaw)
{
	/*机器自旋角速度计算*/
	pid_calc(tg_yaw, now_yaw, &Machine_Beta_PIDpos);
	Chassis_Machine.velW = Machine_Beta_PIDpos.out;
}
/**************************************************************
	@brief:  底盘各轮转速PID运算
	@param:		
	@retval: 		
	@supplement: 1.用于底盘轮子的速度控制
				 2.坐标系为轮子坐标系
**************************************************************/
int Max_Id = 0;
void Wheel_Spdcalc(void)
{
	float wheel_tgspd[3];
	/*************PID计算*************/
	pid_calc(Chassis_Wheel.wheel1, M3508_State_Data[0].spd, &M3508_PIDspd[0]);
	pid_calc(Chassis_Wheel.wheel2, M3508_State_Data[1].spd, &M3508_PIDspd[1]);
	pid_calc(Chassis_Wheel.wheel3, M3508_State_Data[2].spd, &M3508_PIDspd[2]);
	
	/*************获得最大绝对值目标速度*************/
	/*fabs(float/double x),计算浮点数 x 的绝对值*/
	wheel_tgspd[0] = fabs(Chassis_Wheel.wheel1);
	wheel_tgspd[1] = fabs(Chassis_Wheel.wheel2);
	wheel_tgspd[2] = fabs(Chassis_Wheel.wheel3);
	
	Max_Id = Bubble_Sort(wheel_tgspd, 3);//得到输出最大值ID

	/*************计算另外两个轮子速度,使其符合动力学关系，不因PID输出限幅导致变形
	三个轮子给的PID参数相同，但是由于目标值不同，err值不同，导致时刻输出的值不一定符合全向轮运动学关系*************/
	
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
//	// 对PID输出进行指数加权平均平滑处理
//  static float previous_ewma[3] = {0}; // 初始值为0，也可以根据需要初始化
//  M3508_PIDspd[0].out = exponential_weighted_moving_average(M3508_PIDspd[0].out, previous_ewma[0]);
//  M3508_PIDspd[1].out = exponential_weighted_moving_average(M3508_PIDspd[1].out, previous_ewma[1]);
//  M3508_PIDspd[2].out = exponential_weighted_moving_average(M3508_PIDspd[2].out, previous_ewma[2]);
//  // 更新历史EWMA值
//  previous_ewma[0] = M3508_PIDspd[0].out;
//  previous_ewma[1] = M3508_PIDspd[1].out;
//  previous_ewma[2] = M3508_PIDspd[2].out;
	/*************最大绝对值目标速度为0，则其它数应都为0*************/
	if(M3508_PIDspd[Max_Id].out == 0)
	{
		M3508_PIDspd[0].out = 0;
		M3508_PIDspd[1].out = 0;
		M3508_PIDspd[2].out = 0;
	}
}
/**************************************************************
	@brief:  跟踪死区设定
	@param:		
	@retval: 		
	@supplement: 当进入一段视觉死区，停止继续跟踪
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

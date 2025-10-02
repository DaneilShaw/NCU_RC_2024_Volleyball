#include "Vision_Task.h"
/*********************************全局变量*********************************/
KalmanType_t Vision_Kalman_Xaxis;
KalmanType_t Vision_Kalman_Yaxis;

float Kalman_X_axis;
float Kalman_Y_axis;
float Vision_Inertial_Xspd, Vision_Inertial_Yspd;
/*********************************全局函数*********************************/

void Vision_Task(void const * argument)
{
	/*创建卡尔曼滤波器*/
	Onedimensior_KalmanFilter_Create(&Vision_Kalman_Xaxis, 0.1f, 1.0f);
	Onedimensior_KalmanFilter_Create(&Vision_Kalman_Yaxis, 0.1f, 1.0f);
	
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();//获取时钟节拍
	for(;;)
	{
		//更新任务调度时间
		Vision_Moment = Get_Systerm_Time();
		//底盘模式转换
		if(VisionRx_DataBuf[0] == 'H' && VisionRx_DataBuf[1] == 'I' && VisionRx_DataBuf[2] == 'T')
		{
			/*标志位赋值为 制动刹车*/
			Chassis_Mode = Retardation_Mode;
		}
		//对相机反馈的速度数据进行一阶低通滤波,得到滤波过后的速度值
		Vision_Inertial_Xspd = Inertial_Link_Filter(Camera_Coordinate.X_Spd);
		Vision_Inertial_Yspd = Inertial_Link_Filter(Camera_Coordinate.Y_Spd);
		//对位置数据采用卡尔曼滤波平滑处理
		Kalman_X_axis = Onedimensior_Kalman_Filter(&Vision_Kalman_Xaxis, Camera_Coordinate.X_axis, Vision_Inertial_Xspd, 5);
		Kalman_Y_axis = Onedimensior_Kalman_Filter(&Vision_Kalman_Yaxis, Camera_Coordinate.Y_axis, Vision_Inertial_Yspd, 5);
		//清空视觉数据
		memset(VisionRx_DataBuf, 0, 32);
		osDelayUntil(&xLastWakeTime, 5);//绝对延时,其它注意不要出现偶数延时，否则其它任务无法获取CPU权限
	}
}


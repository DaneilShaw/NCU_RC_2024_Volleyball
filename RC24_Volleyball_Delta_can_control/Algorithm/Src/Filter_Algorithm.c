#include "Filter_Algorithm.h"

/*******************创建一维激光卡尔曼滤波器*******************/
void Onedimensior_LaserKalmanFilter_Create(KalmanType_t * Kalman, float tgQ, float tgR)
{
	Kalman->A = 1.0f;
	Kalman->B = 0.0f;
	Kalman->H = 1.0f;
	
	Kalman->Q = tgQ;//过程误差，Q越大，越相信测量值
	Kalman->R = tgR;//测量误差，R越大，越相信预测值
	
	Kalman->P = 1.0f;//可以很快收敛
	Kalman->X = 0.0f;//不知道具体状态，当然一开始可以设置一个接近的状态，直接初始化为测量值
}

/******************对速度进行梯形积分*******************/
float Trapezoidal_Velocity_Integral(KalmanType_t * Kalman, float speed, float period)
{
	static float vel_last, vel;
	Kalman->B = period / 1000;
  vel = speed;
	Kalman->U = (vel_last + vel) / 2;
	float output = Kalman->B * Kalman->U;
	vel_last = vel;
	return output;
}
/*******************一维卡尔曼滤波器*******************/
float Onedimensior_LaserKalman_Filter(KalmanType_t * Kalman, float measure, float speed, float period)
{
	Kalman->Vel_Cal = Trapezoidal_Velocity_Integral(Kalman, speed, period);
	/**************更新************/
	//计算先验状态值
	Kalman->X = Kalman->A * Kalman->X + Kalman->Vel_Cal;
	//计算先验协方差矩阵
	Kalman->P = Kalman->P + Kalman->Q;
	/**************矫正************/
	//计算卡尔曼增益，此处由于是一维矩阵，设置观测值与状态值的转换为1
	Kalman->K = Kalman->P / (Kalman->P + Kalman->R);
	//融合观测值和先验值，获得后验状态值
	Kalman->X = Kalman->X + Kalman->K *(measure - Kalman->H * Kalman->X);
	//更新协方差矩阵
	Kalman->P = (1 - Kalman->K * Kalman->H) * Kalman->P;
	
	return  Kalman->X;
}



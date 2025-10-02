#include "Filter_Algorithm.h"

/*******************创建一维卡尔曼滤波器*******************/
void Onedimensior_KalmanFilter_Create(KalmanType_t * Kalman, float tgQ, float tgR)
{
	Kalman->A = 1.0f;//一维卡尔曼滤波，状态转移矩阵也为1
	Kalman->B = 0.0f;
	Kalman->H = 1.0f;//一维卡尔曼滤波，状态量到观测量的转换矩阵为1
	
	Kalman->Q = tgQ;//过程误差(系统过程误差)，Q越大，越相信测量值
	Kalman->R = tgR;//测量误差(传感器测量误差)，R越大，越相信预测值
	
	Kalman->P = 1.0f;//可以很快收敛
	Kalman->X = 0.0f;//不知道具体状态，当然一开始可以设置一个接近的状态，直接初始化为测量值
}

/******************对速度进行梯形积分*******************/
float Trapezoidal_Velocity_Integral(KalmanType_t * Kalman, float speed, float period)
{
	static float vel_last, vel;
	Kalman->B = period / 1000;
  vel = speed;	//获取当前速度
	Kalman->U = (vel_last + vel) / 2; 	//计算上一次速度和这一次速度的平均值，即梯形上底和下底的平均长度
	float output = Kalman->B * Kalman->U; 	//将平均速度*时间 = 近似位移
	vel_last = vel;
	return output;
}

/*******************一维卡尔曼滤波器*******************/
float Onedimensior_Kalman_Filter(KalmanType_t * Kalman, float measure, float speed, float period)
{
	//对刚低通滤波完的速度数据经行梯形积分得到新的位置数据
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
/**************************************************************
	@brief:		一阶惯性环节的时间离散模型进行滤波，对相机反馈的速度数据进行一阶低通滤波
	@param:		使用静态变量 last_output 记忆上一次滤波的输出值，
	@retval: 		
	@supplement:	用于平滑突变信号，过滤高频噪声
                1.平滑阶跃信号
                2.过滤高频噪声，如微分计算后的数据
**************************************************************/
float Inertial_Link_Filter(float input)
{
	static float update_output, last_output;
	float T = 0.1f;//惯性环节时间常数，越大收敛越慢
	/*T：时间常数，它决定了滤波器对输入信号变化的响应速度*/
	/*时间常数T越大，滤波器的响应越慢，对高频信号的过滤效果越好*/
	float dt = 0.005f;//采样时间间隔
	
	update_output = (input * dt + last_output * (T - dt)) / T;
	/*公式实际上是一阶惯性环节的离散时间模型，它结合了当前输入和上一次的输出，
		通过时间常数 T 来控制滤波器的响应*/
	/* 此刻的输出值 = (此刻的输入值 * dt + 上一刻的输出值 * (T - dt))/T		*/
	/* 通过当前的输入值和上一时刻的输出值，加权求和并归一化，来计算当前时刻的输出值	*/
	last_output = update_output;
	return update_output;
}

/**************************************************************
	@brief:		滑动平均滤波
	@param:		
	@retval: 		
	@supplement:	用于信号的平滑处理
**************************************************************/
float Move_Average_Filter(float * data)
{
	static float window_buf[5], sum;
	static int buf_index = 0;
	float average;
	if(buf_index == 5)
	{
		sum = sum - window_buf[0];
		for(int i = 0; i <= 3; i++)
		{
			window_buf[i] = window_buf[i+1];
		}
		window_buf[4] = *data;
		sum = sum + window_buf[4];
	}
	else if(buf_index < 5)
	{
		window_buf[buf_index] = *data;
		sum += window_buf[buf_index];
		buf_index++;
	}
	average = sum / buf_index;
	return average;
}
/**************************************************************
	@brief:		指数加权平均滤波器
	@param:		
	@retval: 		
	@supplement:	用于信号的平滑处理
**************************************************************/
float exponential_weighted_moving_average(float new_value, float previous_ewma) 
{
	return ALPHA * new_value + (1 - ALPHA) * previous_ewma;
}



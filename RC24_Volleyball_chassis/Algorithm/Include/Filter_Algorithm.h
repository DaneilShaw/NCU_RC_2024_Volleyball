#ifndef __FILTER_ALGORITHM_H
#define __FILTER_ALGORITHM_H

#define ALPHA 0.9f  // 定义平滑因子，取值范围为0到1

typedef struct
{
	float X;//估计值
	float A;//与X有关的系数矩阵
	float U;//控制量
	float B;//与控制量相关矩阵
	float Q;//过程噪声协方差，Q增大，动态响应变快，收敛稳定性变坏
	
	float H;//观测量与状态量的关系矩阵
	float R;//测量噪声协方差，R增大，动态响应变慢，收敛稳定性变好
	
	float P;//估算协方差
	float K;//卡尔曼增益
	float Output;//卡尔曼滤波器输出

//用于激光预测模型的控制量进行梯形积分	
	float Vel_Cal;//预测模型计算速度
}KalmanType_t;

extern void Onedimensior_KalmanFilter_Create(KalmanType_t * Kalman, float tgQ, float tgR);
extern float Onedimensior_Kalman_Filter(KalmanType_t * Kalman, float measure, float speed, float period);
extern float Inertial_Link_Filter(float input);
extern float Move_Average_Filter(float * data);
extern float exponential_weighted_moving_average(float new_value, float previous_ewma);


#endif


#include "Filter_Algorithm.h"

/*******************����һά���⿨�����˲���*******************/
void Onedimensior_LaserKalmanFilter_Create(KalmanType_t * Kalman, float tgQ, float tgR)
{
	Kalman->A = 1.0f;
	Kalman->B = 0.0f;
	Kalman->H = 1.0f;
	
	Kalman->Q = tgQ;//������QԽ��Խ���Ų���ֵ
	Kalman->R = tgR;//������RԽ��Խ����Ԥ��ֵ
	
	Kalman->P = 1.0f;//���Ժܿ�����
	Kalman->X = 0.0f;//��֪������״̬����Ȼһ��ʼ��������һ���ӽ���״̬��ֱ�ӳ�ʼ��Ϊ����ֵ
}

/******************���ٶȽ������λ���*******************/
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
/*******************һά�������˲���*******************/
float Onedimensior_LaserKalman_Filter(KalmanType_t * Kalman, float measure, float speed, float period)
{
	Kalman->Vel_Cal = Trapezoidal_Velocity_Integral(Kalman, speed, period);
	/**************����************/
	//��������״ֵ̬
	Kalman->X = Kalman->A * Kalman->X + Kalman->Vel_Cal;
	//��������Э�������
	Kalman->P = Kalman->P + Kalman->Q;
	/**************����************/
	//���㿨�������棬�˴�������һά�������ù۲�ֵ��״ֵ̬��ת��Ϊ1
	Kalman->K = Kalman->P / (Kalman->P + Kalman->R);
	//�ںϹ۲�ֵ������ֵ����ú���״ֵ̬
	Kalman->X = Kalman->X + Kalman->K *(measure - Kalman->H * Kalman->X);
	//����Э�������
	Kalman->P = (1 - Kalman->K * Kalman->H) * Kalman->P;
	
	return  Kalman->X;
}



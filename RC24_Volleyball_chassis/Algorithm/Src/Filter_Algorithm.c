#include "Filter_Algorithm.h"

/*******************����һά�������˲���*******************/
void Onedimensior_KalmanFilter_Create(KalmanType_t * Kalman, float tgQ, float tgR)
{
	Kalman->A = 1.0f;//һά�������˲���״̬ת�ƾ���ҲΪ1
	Kalman->B = 0.0f;
	Kalman->H = 1.0f;//һά�������˲���״̬�����۲�����ת������Ϊ1
	
	Kalman->Q = tgQ;//�������(ϵͳ�������)��QԽ��Խ���Ų���ֵ
	Kalman->R = tgR;//�������(�������������)��RԽ��Խ����Ԥ��ֵ
	
	Kalman->P = 1.0f;//���Ժܿ�����
	Kalman->X = 0.0f;//��֪������״̬����Ȼһ��ʼ��������һ���ӽ���״̬��ֱ�ӳ�ʼ��Ϊ����ֵ
}

/******************���ٶȽ������λ���*******************/
float Trapezoidal_Velocity_Integral(KalmanType_t * Kalman, float speed, float period)
{
	static float vel_last, vel;
	Kalman->B = period / 1000;
  vel = speed;	//��ȡ��ǰ�ٶ�
	Kalman->U = (vel_last + vel) / 2; 	//������һ���ٶȺ���һ���ٶȵ�ƽ��ֵ���������ϵ׺��µ׵�ƽ������
	float output = Kalman->B * Kalman->U; 	//��ƽ���ٶ�*ʱ�� = ����λ��
	vel_last = vel;
	return output;
}

/*******************һά�������˲���*******************/
float Onedimensior_Kalman_Filter(KalmanType_t * Kalman, float measure, float speed, float period)
{
	//�Ըյ�ͨ�˲�����ٶ����ݾ������λ��ֵõ��µ�λ������
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
/**************************************************************
	@brief:		һ�׹��Ի��ڵ�ʱ����ɢģ�ͽ����˲���������������ٶ����ݽ���һ�׵�ͨ�˲�
	@param:		ʹ�þ�̬���� last_output ������һ���˲������ֵ��
	@retval: 		
	@supplement:	����ƽ��ͻ���źţ����˸�Ƶ����
                1.ƽ����Ծ�ź�
                2.���˸�Ƶ��������΢�ּ���������
**************************************************************/
float Inertial_Link_Filter(float input)
{
	static float update_output, last_output;
	float T = 0.1f;//���Ի���ʱ�䳣����Խ������Խ��
	/*T��ʱ�䳣�������������˲����������źű仯����Ӧ�ٶ�*/
	/*ʱ�䳣��TԽ���˲�������ӦԽ�����Ը�Ƶ�źŵĹ���Ч��Խ��*/
	float dt = 0.005f;//����ʱ����
	
	update_output = (input * dt + last_output * (T - dt)) / T;
	/*��ʽʵ������һ�׹��Ի��ڵ���ɢʱ��ģ�ͣ�������˵�ǰ�������һ�ε������
		ͨ��ʱ�䳣�� T �������˲�������Ӧ*/
	/* �˿̵����ֵ = (�˿̵�����ֵ * dt + ��һ�̵����ֵ * (T - dt))/T		*/
	/* ͨ����ǰ������ֵ����һʱ�̵����ֵ����Ȩ��Ͳ���һ���������㵱ǰʱ�̵����ֵ	*/
	last_output = update_output;
	return update_output;
}

/**************************************************************
	@brief:		����ƽ���˲�
	@param:		
	@retval: 		
	@supplement:	�����źŵ�ƽ������
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
	@brief:		ָ����Ȩƽ���˲���
	@param:		
	@retval: 		
	@supplement:	�����źŵ�ƽ������
**************************************************************/
float exponential_weighted_moving_average(float new_value, float previous_ewma) 
{
	return ALPHA * new_value + (1 - ALPHA) * previous_ewma;
}



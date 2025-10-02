#ifndef __FILTER_ALGORITHM_H
#define __FILTER_ALGORITHM_H

#define ALPHA 0.9f  // ����ƽ�����ӣ�ȡֵ��ΧΪ0��1

typedef struct
{
	float X;//����ֵ
	float A;//��X�йص�ϵ������
	float U;//������
	float B;//���������ؾ���
	float Q;//��������Э���Q���󣬶�̬��Ӧ��죬�����ȶ��Ա仵
	
	float H;//�۲�����״̬���Ĺ�ϵ����
	float R;//��������Э���R���󣬶�̬��Ӧ�����������ȶ��Ա��
	
	float P;//����Э����
	float K;//����������
	float Output;//�������˲������

//���ڼ���Ԥ��ģ�͵Ŀ������������λ���	
	float Vel_Cal;//Ԥ��ģ�ͼ����ٶ�
}KalmanType_t;

extern void Onedimensior_KalmanFilter_Create(KalmanType_t * Kalman, float tgQ, float tgR);
extern float Onedimensior_Kalman_Filter(KalmanType_t * Kalman, float measure, float speed, float period);
extern float Inertial_Link_Filter(float input);
extern float Move_Average_Filter(float * data);
extern float exponential_weighted_moving_average(float new_value, float previous_ewma);


#endif


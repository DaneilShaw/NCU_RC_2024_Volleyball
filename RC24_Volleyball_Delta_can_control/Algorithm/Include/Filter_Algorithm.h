#ifndef __FILTER_ALGORITHM_H
#define __FILTER_ALGORITHM_H

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

void Onedimensior_LaserKalmanFilter_Create(KalmanType_t * Kalman, float tgQ, float tgR);
float Onedimensior_LaserKalman_Filter(KalmanType_t * Kalman, float measure, float speed, float period);


#endif


#include "Vision_Task.h"
/*********************************ȫ�ֱ���*********************************/
KalmanType_t Vision_Kalman_Xaxis;
KalmanType_t Vision_Kalman_Yaxis;

float Kalman_X_axis;
float Kalman_Y_axis;
float Vision_Inertial_Xspd, Vision_Inertial_Yspd;
/*********************************ȫ�ֺ���*********************************/

void Vision_Task(void const * argument)
{
	/*�����������˲���*/
	Onedimensior_KalmanFilter_Create(&Vision_Kalman_Xaxis, 0.1f, 1.0f);
	Onedimensior_KalmanFilter_Create(&Vision_Kalman_Yaxis, 0.1f, 1.0f);
	
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();//��ȡʱ�ӽ���
	for(;;)
	{
		//�����������ʱ��
		Vision_Moment = Get_Systerm_Time();
		//����ģʽת��
		if(VisionRx_DataBuf[0] == 'H' && VisionRx_DataBuf[1] == 'I' && VisionRx_DataBuf[2] == 'T')
		{
			/*��־λ��ֵΪ �ƶ�ɲ��*/
			Chassis_Mode = Retardation_Mode;
		}
		//������������ٶ����ݽ���һ�׵�ͨ�˲�,�õ��˲�������ٶ�ֵ
		Vision_Inertial_Xspd = Inertial_Link_Filter(Camera_Coordinate.X_Spd);
		Vision_Inertial_Yspd = Inertial_Link_Filter(Camera_Coordinate.Y_Spd);
		//��λ�����ݲ��ÿ������˲�ƽ������
		Kalman_X_axis = Onedimensior_Kalman_Filter(&Vision_Kalman_Xaxis, Camera_Coordinate.X_axis, Vision_Inertial_Xspd, 5);
		Kalman_Y_axis = Onedimensior_Kalman_Filter(&Vision_Kalman_Yaxis, Camera_Coordinate.Y_axis, Vision_Inertial_Yspd, 5);
		//����Ӿ�����
		memset(VisionRx_DataBuf, 0, 32);
		osDelayUntil(&xLastWakeTime, 5);//������ʱ,����ע�ⲻҪ����ż����ʱ���������������޷���ȡCPUȨ��
	}
}


#ifndef __SABER_INERTIA_H
#define __SABER_INERTIA_H

#include "struct_typedef.h"
#include "usart.h"
#include "bsp_usart.h"
#include "wheel_calculation.h"

/***********************************�ߵ�ģʽ***********************************/
enum mode
{
	WAKEUP_MODE,
	CONFIG_MODE,
	MEASURE_MODE,
	SELFTEST_MODE,
	MAG_CAL_MODE
};

/***********************************CID***********************************/
typedef enum
{
	CLASS_ID_OPERATION_CMD = 0x01, //�����ࣺ�ṩ����ģ�����;��
	CLASS_ID_DEVICE_INFO = 0x02,   //�豸��Ϣ�ࣺ�ṩ��ȡ�豸��Ϣ�ķ���
	CLASS_ID_SENSOR_CONFIG = 0x03, //�����������ࣺ�������ô�����
	CLASS_ID_ALGORITHMENGINE = 0x04, //�㷨�����ࣺ��������Ĺ���ģʽ�Ͳ�������������
	CLASS_ID_COMMUNICATION_CONFIG = 0x05, //ͨ�������ࣺ�ṩһЩ��������ͨ�Žӿ�
	CLASS_ID_HOSTCONTROL_CMD = 0x06, //���������ࣺ���������������������������ź�
	CLASS_ID_FIRMWARE_UPDATE = 0x0A, //�̼������ࣺ�����ṩ���¹̼��ķ���
	CLASS_ID_DEBUG = 0x0E, //�����ࣺ�����ڲ�����
}Class_ID;//CID

/***********************************PID***********************************/
typedef enum
{
	Temperature_Enabel_ID = 0x8000,  //�¶����ݰ�PID
	
	RawAcc_Enable_ID = 0x8400,       //���ٶȼ�ԭʼ����(X/Y/Z��)PID����λg
	RawGyro_Enable_ID = 0x8401,      //������ԭʼ����(X/Y/Z��)PID����λdps
	RawMag_Enable_ID = 0x8402,       //������ԭʼ����(X/Y/Z��)PID����λmGauss
	RawBaro_Enable_ID = 0x8403,      //��ѹ��ԭʼ����
	
	CalAcc_Enable_ID = 0x8800,       //���ٶȼ�У׼����PID
	KalmanAcc_Enable_ID = 0x8801,    //���ٶȼƿ������˲�����PID
	
	CalGyro_Enable_ID = 0x8C00,      //������У׼����PID
	KalmanGyro_Enable_ID = 0x8C01,   //�����ǿ������˲�����PID
	
	CalMag_Enable_ID = 0x9000,       //������У׼����PID
	KalmanMag_Enable_ID = 0x9001,    //�����ƿ������˲�����PID
	MagDeviation_Enable_ID = 0x9002, //������ƫ������PID

	KalBarometer_Enable_ID = 0x9401, //�������˲�����ѹ������PID
	
	GNSS_Enable_ID = 0xA000,         //Gnss PVTԭʼ����
	GnssSate_Enable_ID = 0xA001,     //��Gnss�����������������
	
//	GPS_Enable_ID = 0xA400,
	
	Quaternion_Enable_ID = 0xB000,   //��Ԫ��
	CapeEuler_Enable_ID = 0xB001,    //ŷ����
	Cos_Enable_ID = 0xB002,          //���Ҿ���
	
	Position_Enable_ID = 0xB403,     //λ������
	Alti_Ellipsoid_Enable_ID = 0xB400,//�߶�����
	Lonlat_Enable_ID = 0xB402,       //����γ������
	Odometer_Enable_ID = 0xB500,     //���ټ�����
	
	LinearAcc_Enable_ID = 0xB800,    //���Լ��ٶ�����PID����λm/s^2
	Velocity_Enable_ID = 0xB802,     //�ٶ�����PID����λm/s
	HeaveMotion_Enable_ID = 0xB803,  //��������PID����λm
	
	Package_Enable_ID = 0xC000,      //���ݰ����PID
	Time_Enable_ID = 0xC002,         //����ϵͳ�ο�ʱ��PID
	SampInterval_Enable_ID = 0xC006, //�������PID
	Status_Enable_ID = 0xC400,       //״̬��PID
}Saber_PID;

/***********************************ԭʼ����***********************************/
typedef struct
{
	short RawXaxis_Data;
	short RawYaxis_Data;
	short RawZaxis_Data;
}RawData;
/***********************************У׼����***********************************/
typedef struct
{
	float CalXaxis_Data;  //Bite0-3
	float CalYaxis_Data;  //Bite4-7
	float CalZaxis_Data;  //Bite8-11
}Cal_Data;
/***********************************�������˲�����***********************************/
typedef struct
{
	float Xaxis_Data;  //Bite0-3
	float Yaxis_Data;  //Bite4-7
	float Zaxis_Data;  //Bite8-11
}Kal_Data;

/***********************************ŷ��������***********************************/
typedef struct
{
	float roll;  //Bite0-3
	float pitch;  //Bite4-7
	float yaw;  //Bite8-11
}CapeEuler_Data_t;//roll��B0-B3��pitch��B4-B7��yaw��B8-B11

/***********************************������Ϣ���ø�ʽ***********************************/
typedef struct
{
	//ͷ������
	uint8_t Preamble[2];   //ͷ�����ֽ�ΪAx
	uint8_t MADDR;         //��ַ���0-0xFE���㲥��Ϣ��0-0xFF
	uint8_t CID;           //����λΪ0��ʾ�޴��󣬵���λ�ο�Class_ID
	uint8_t MID;           //��ӦCIDָ���µĲ�ͬ������
	uint8_t PL;            //��Ч�غɳ���0-0xFE,0xFF����������֧�ֳ������ݰ�(SLP)
	uint16_t SLP;          //255-1024��SLP��0xFFʱ��Ч��2�ֽ�
	
	//��Ϣ����
	uint8_t Reserve1;       //0xFF
	uint8_t Reserve2;       //0xFF
	uint16_t PID;           //��ӦID,��Saber_ID�е�ID
	
	//β������
	uint8_t Atom_BCC;      //��ЧУ��λ1�ֽ�  
	uint8_t TALL;          //β����ֹ��0x6D
}Saber_Config_t;//���������������豸

/***********************************���ݰ����ݸ�ʽ***********************************/
typedef struct
{
	uint16_t RxPID;          //�ߵ����͸����ص�PID,��Ӧaber_ID�е�ID
	uint8_t RxPL;            //���յ������ݳ���
	float Temperature;
	
	RawData Raw_Data;        //Acc��Gyro��Magԭ������,X:B0-B1,Y:B2-B3,Z:B4-B5
  Cal_Data Calibrated_Data;//Acc��Gyro��MagУ׼����,X:B0-B3,Y:B4-B7,Z:B8-B11
	Kal_Data Kalman_Data;    //Acc��Gyro��Mag�������˲�����,X:B0-B3,Y:B4-B7,Z:B8-B11
	
	float MagDev_Data;       //����ƫ������B0-B3
	int32_t RawBaro_Data;    //��ѹ��ԭʼ���ݣ���λ��˹����B0-B3
	int32_t KalmanBaro_Data;    //��ѹ�ƿ������˲����ݣ���λ��˹����B0-B3
	
	float Quaternion_Data[4];   //	Q0��B0-B3��Q1��B4-B7��Q2��B8-B11��Q3��B12-B15
	CapeEuler_Data_t Euler_Data;    //roll��B0-B3��pitch��B4-B7��yaw��B8-B11
	
	float LinearAcc_Data[3];    //���Լ��ٶ����ݣ�X:B0-B3,Y:B4-B7,B8-B11,m/s^2
	float Velocity_Data[3];    //�ٶ����ݣ�X:B0-B3,Y:B4-B7,B8-B11,m/s
	float Position_Data[3];    //λ�����ݣ�X:B0-B3,Y:B4-B7,B8-B11,m
	float Heave_Data;       //�������ݣ�B0-B4����λm
	float Altitude_Data;    //���θ߶ȣ�B0-B4
	float Odometer_Data;    //��̼����ݣ�LS:B0-B3,RS:B4-B7,LD:B8-B11,B12-B15
	
	uint8_t package_Data;   //���ݰ��������B0-B4
	float SampleTime_Data;  //�������ʱ��B0-B4
	float Status_Data;      //��ǰϵͳ״̬��B0-B4
}Saber_Packet_Data;

/***********************************���ݰ��ṹ***********************************/
typedef struct
{
	uint16_t PID;       //���ݶ�ӦPID
	uint8_t PL;         //��ȡ���ݵ���Ч�غɳ���
//	Saber_Packet_Data Saber_RxData;       //��ӦID,��Saber_ID�е�ID
}Saber_Package;//�����豸����������

extern void SaberInertia_USART6_RxCpltCallBack(void);

#endif


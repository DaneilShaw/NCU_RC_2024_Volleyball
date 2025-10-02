#ifndef __SABER_INERTIA_H
#define __SABER_INERTIA_H

#include "struct_typedef.h"
#include "usart.h"
#include "bsp_usart.h"
#include "wheel_calculation.h"

/***********************************惯导模式***********************************/
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
	CLASS_ID_OPERATION_CMD = 0x01, //操作类：提供控制模组基本途径
	CLASS_ID_DEVICE_INFO = 0x02,   //设备信息类：提供获取设备信息的方法
	CLASS_ID_SENSOR_CONFIG = 0x03, //传感器配置类：用于配置传感器
	CLASS_ID_ALGORITHMENGINE = 0x04, //算法引擎类：配置引擎的工作模式和参数来控制引擎
	CLASS_ID_COMMUNICATION_CONFIG = 0x05, //通信配置类：提供一些命令配置通信接口
	CLASS_ID_HOSTCONTROL_CMD = 0x06, //主机控制类：用于配置主机处理器的其余项信号
	CLASS_ID_FIRMWARE_UPDATE = 0x0A, //固件升级类：用于提供更新固件的方法
	CLASS_ID_DEBUG = 0x0E, //调试类：用于内部测试
}Class_ID;//CID

/***********************************PID***********************************/
typedef enum
{
	Temperature_Enabel_ID = 0x8000,  //温度数据包PID
	
	RawAcc_Enable_ID = 0x8400,       //加速度计原始数据(X/Y/Z轴)PID，单位g
	RawGyro_Enable_ID = 0x8401,      //陀螺仪原始数据(X/Y/Z轴)PID，单位dps
	RawMag_Enable_ID = 0x8402,       //磁力计原始数据(X/Y/Z轴)PID，单位mGauss
	RawBaro_Enable_ID = 0x8403,      //气压计原始数据
	
	CalAcc_Enable_ID = 0x8800,       //加速度计校准数据PID
	KalmanAcc_Enable_ID = 0x8801,    //加速度计卡尔曼滤波数据PID
	
	CalGyro_Enable_ID = 0x8C00,      //陀螺仪校准数据PID
	KalmanGyro_Enable_ID = 0x8C01,   //陀螺仪卡尔曼滤波数据PID
	
	CalMag_Enable_ID = 0x9000,       //磁力计校准数据PID
	KalmanMag_Enable_ID = 0x9001,    //磁力计卡尔曼滤波数据PID
	MagDeviation_Enable_ID = 0x9002, //磁力计偏差数据PID

	KalBarometer_Enable_ID = 0x9401, //卡尔曼滤波的气压计数据PID
	
	GNSS_Enable_ID = 0xA000,         //Gnss PVT原始数据
	GnssSate_Enable_ID = 0xA001,     //带Gnss工程输出的卫星数据
	
//	GPS_Enable_ID = 0xA400,
	
	Quaternion_Enable_ID = 0xB000,   //四元数
	CapeEuler_Enable_ID = 0xB001,    //欧拉角
	Cos_Enable_ID = 0xB002,          //余弦矩阵
	
	Position_Enable_ID = 0xB403,     //位移数据
	Alti_Ellipsoid_Enable_ID = 0xB400,//高度数据
	Lonlat_Enable_ID = 0xB402,       //经度纬度数据
	Odometer_Enable_ID = 0xB500,     //轮速计数据
	
	LinearAcc_Enable_ID = 0xB800,    //线性加速度数据PID，单位m/s^2
	Velocity_Enable_ID = 0xB802,     //速度数据PID，单位m/s
	HeaveMotion_Enable_ID = 0xB803,  //升沉数据PID，单位m
	
	Package_Enable_ID = 0xC000,      //数据包编号PID
	Time_Enable_ID = 0xC002,         //操作系统参考时间PID
	SampInterval_Enable_ID = 0xC006, //采样间隔PID
	Status_Enable_ID = 0xC400,       //状态字PID
}Saber_PID;

/***********************************原始数据***********************************/
typedef struct
{
	short RawXaxis_Data;
	short RawYaxis_Data;
	short RawZaxis_Data;
}RawData;
/***********************************校准数据***********************************/
typedef struct
{
	float CalXaxis_Data;  //Bite0-3
	float CalYaxis_Data;  //Bite4-7
	float CalZaxis_Data;  //Bite8-11
}Cal_Data;
/***********************************卡尔曼滤波数据***********************************/
typedef struct
{
	float Xaxis_Data;  //Bite0-3
	float Yaxis_Data;  //Bite4-7
	float Zaxis_Data;  //Bite8-11
}Kal_Data;

/***********************************欧拉角数据***********************************/
typedef struct
{
	float roll;  //Bite0-3
	float pitch;  //Bite4-7
	float yaw;  //Bite8-11
}CapeEuler_Data_t;//roll：B0-B3，pitch：B4-B7，yaw：B8-B11

/***********************************数据消息配置格式***********************************/
typedef struct
{
	//头部数据
	uint8_t Preamble[2];   //头两个字节为Ax
	uint8_t MADDR;         //地址编号0-0xFE，广播信息用0-0xFF
	uint8_t CID;           //高四位为0表示无错误，低四位参考Class_ID
	uint8_t MID;           //对应CID指令下的不同子命令
	uint8_t PL;            //有效载荷长度0-0xFE,0xFF被保留用来支持超长数据包(SLP)
	uint16_t SLP;          //255-1024，SLP在0xFF时有效，2字节
	
	//消息数据
	uint8_t Reserve1;       //0xFF
	uint8_t Reserve2;       //0xFF
	uint16_t PID;           //对应ID,即Saber_ID中的ID
	
	//尾部数据
	uint8_t Atom_BCC;      //有效校验位1字节  
	uint8_t TALL;          //尾部终止码0x6D
}Saber_Config_t;//用于主机发送至设备

/***********************************数据包内容格式***********************************/
typedef struct
{
	uint16_t RxPID;          //惯导发送给主控的PID,对应aber_ID中的ID
	uint8_t RxPL;            //接收到的数据长度
	float Temperature;
	
	RawData Raw_Data;        //Acc、Gyro、Mag原生数据,X:B0-B1,Y:B2-B3,Z:B4-B5
  Cal_Data Calibrated_Data;//Acc、Gyro、Mag校准数据,X:B0-B3,Y:B4-B7,Z:B8-B11
	Kal_Data Kalman_Data;    //Acc、Gyro、Mag卡尔曼滤波数据,X:B0-B3,Y:B4-B7,Z:B8-B11
	
	float MagDev_Data;       //磁力偏差数据B0-B3
	int32_t RawBaro_Data;    //气压计原始数据，单位帕斯卡，B0-B3
	int32_t KalmanBaro_Data;    //气压计卡尔曼滤波数据，单位帕斯卡，B0-B3
	
	float Quaternion_Data[4];   //	Q0：B0-B3，Q1：B4-B7，Q2：B8-B11，Q3：B12-B15
	CapeEuler_Data_t Euler_Data;    //roll：B0-B3，pitch：B4-B7，yaw：B8-B11
	
	float LinearAcc_Data[3];    //线性加速度数据，X:B0-B3,Y:B4-B7,B8-B11,m/s^2
	float Velocity_Data[3];    //速度数据，X:B0-B3,Y:B4-B7,B8-B11,m/s
	float Position_Data[3];    //位移数据，X:B0-B3,Y:B4-B7,B8-B11,m
	float Heave_Data;       //升沉数据，B0-B4，单位m
	float Altitude_Data;    //海拔高度，B0-B4
	float Odometer_Data;    //里程计数据，LS:B0-B3,RS:B4-B7,LD:B8-B11,B12-B15
	
	uint8_t package_Data;   //数据包编号数据B0-B4
	float SampleTime_Data;  //采样间隔时间B0-B4
	float Status_Data;      //当前系统状态字B0-B4
}Saber_Packet_Data;

/***********************************数据包结构***********************************/
typedef struct
{
	uint16_t PID;       //数据对应PID
	uint8_t PL;         //获取数据的有效载荷长度
//	Saber_Packet_Data Saber_RxData;       //对应ID,即Saber_ID中的ID
}Saber_Package;//用于设备发送至主机

extern void SaberInertia_USART6_RxCpltCallBack(void);

#endif


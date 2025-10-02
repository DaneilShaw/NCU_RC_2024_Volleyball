#ifndef __GM6020_H
#define __GM6020_H
#include "can.h"
#include "pid.h"

extern pid_t GM6020_PID;

#define voltage1_identifier 0x1FF //电压控制模式，反馈报文标识符：0x205-0x208
#define voltage2_identifier 0x2FF //电压控制模式，反馈报文标识符：0x209-0x20B
#define current1_identifier 0x1FE //电流控制模式，反馈报文标识符：0x205-0x208
#define current2_identifier 0x2FE //电流控制模式，反馈报文标识符：0x209-0x20B

typedef struct
{
	int32_t targetpos;       //目标位置值
	uint16_t pos;             //反馈的现在位置值(0-8191)
	uint16_t lastpos;         //反馈的上一次位置值(0-8191)
	uint16_t basepos;         //转子初始位置
	int32_t num;             //圈数
	int32_t sumpos;          //圈数位置+基本位置，即转子所处转角角度(>8191)
	
	int16_t targetspd;       //目标速度
	int16_t spd;             //速度
	
	int16_t targetcurrent;   //目标电流
	int16_t current;         //电流
	
	uint8_t temp;            //温度
} DjiMotor_State;



extern void GM6020_Write(DjiMotor_State * ptr_State, float targetpos, float targetspd, float targetcurrent);
extern void Get_Nowpos(DjiMotor_State * ptr_State);
extern void GM6020_CAN_cmd(uint32_t identifier, int16_t yaw);
extern void Get_Basepos(DjiMotor_State * ptr);

void M3508_CAN_cmd(uint32_t identifier, int16_t wheel1, int16_t wheel2, int16_t wheel3);

#endif


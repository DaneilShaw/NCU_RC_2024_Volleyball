#ifndef __GM6020_H
#define __GM6020_H
#include "can.h"
#include "pid.h"

extern pid_t GM6020_PID;

#define voltage1_identifier 0x1FF //��ѹ����ģʽ���������ı�ʶ����0x205-0x208
#define voltage2_identifier 0x2FF //��ѹ����ģʽ���������ı�ʶ����0x209-0x20B
#define current1_identifier 0x1FE //��������ģʽ���������ı�ʶ����0x205-0x208
#define current2_identifier 0x2FE //��������ģʽ���������ı�ʶ����0x209-0x20B

typedef struct
{
	int32_t targetpos;       //Ŀ��λ��ֵ
	uint16_t pos;             //����������λ��ֵ(0-8191)
	uint16_t lastpos;         //��������һ��λ��ֵ(0-8191)
	uint16_t basepos;         //ת�ӳ�ʼλ��
	int32_t num;             //Ȧ��
	int32_t sumpos;          //Ȧ��λ��+����λ�ã���ת������ת�ǽǶ�(>8191)
	
	int16_t targetspd;       //Ŀ���ٶ�
	int16_t spd;             //�ٶ�
	
	int16_t targetcurrent;   //Ŀ�����
	int16_t current;         //����
	
	uint8_t temp;            //�¶�
} DjiMotor_State;



extern void GM6020_Write(DjiMotor_State * ptr_State, float targetpos, float targetspd, float targetcurrent);
extern void Get_Nowpos(DjiMotor_State * ptr_State);
extern void GM6020_CAN_cmd(uint32_t identifier, int16_t yaw);
extern void Get_Basepos(DjiMotor_State * ptr);

void M3508_CAN_cmd(uint32_t identifier, int16_t wheel1, int16_t wheel2, int16_t wheel3);

#endif


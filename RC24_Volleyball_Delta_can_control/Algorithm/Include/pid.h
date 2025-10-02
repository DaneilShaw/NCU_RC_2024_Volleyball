#ifndef __PID_H
#define __PID_H
#include "struct_typedef.h"
#include "main.h"
#include "math_formula.h"
#include "stdlib.h"

#define Speed_Mode    1
#define Position_Mode 2

enum
{
	Now,
	Last,
};

typedef struct
{
	uint8_t mode;
	
	float kp; //����
	float ki; //΢��
	float kd; //����
	
	float err[2];
	
	float pout;
	float iout;
	float dout;
	float out;
	
	float Integrater_limitation;
	float Output_limitation;
	
//ǰ����ֵ  
	float forwardfeed_value;
	float last_input;
	float previous_input;
//���ַ���	
	float epsilon;
//���ֱ��� 
	float maximum;
	float minimum;
//����ȫ΢��
  float alpha;
	float dout_last;
}pid_t;

//extern pid_t M3508_PID_Planspd[3];
extern pid_t M3508_PID_spd[4];
extern pid_t M3508_PID_UPpos[4];
extern pid_t M3508_PID_Downpos[4];

extern pid_t M3508_PID_Cushion_UPpos[4];
extern pid_t M3508_PID_Hit_UPpos[4];
extern pid_t M3508_PID_Hit_Downpos[4];

extern void pid_init(pid_t *pid, uint8_t mode, float KP, float KI, float KD, float IMaxOut, float MaxOut);
extern void pid_calc(float set, float get, pid_t *pid);
extern float Firstorder_Forwardfeed_Value(pid_t *pid, float input, float dt, float J, float B);
extern void pid_isept_calc(float set, float get, pid_t *pid);
extern void pid_antiwindup_calc(float set, float get, pid_t *pid);
extern void pid_tinter_calc(float set, float get, pid_t *pid);
extern void pid_incompletediff_calc(float set, float get, pid_t *pid);

#endif


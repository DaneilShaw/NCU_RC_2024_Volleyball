#ifndef __PID_H
#define __PID_H
#include "struct_typedef.h"

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
	
	float kp; //比例
	float ki; //微分
	float kd; //积分
	
	float err[2];
	
	float pout;
	float iout;
	float dout;
	float out;
	
	float Integrater_limitation;
	float Output_limitation;
	
//前馈数值  
	float forwardfeed_value;
	float last_input;
//积分分离	
	float epsilon;
//积分饱和 
	float maximum;
	float minimum;
//不完全微分
  float alpha;
	float dout_last;
}pid_t;

extern pid_t GM6020_PIDpos;
extern pid_t GM6020_PIDspd;

extern pid_t World_Chassis_X_PIDpos;
extern pid_t World_Chassis_Y_PIDpos;
extern pid_t Machine_Beta_PIDpos;
extern pid_t M3508_PIDspd[3];
extern pid_t M3508_PIDpos[3];

extern void pid_init(pid_t *pid, uint8_t mode, float KP, float KI, float KD, float IMaxOut, float MaxOut);
extern void pid_calc(float set, float get, pid_t *pid);
extern float Forwardfeed_Value(pid_t *pid, float input, float dt);
extern void pid_isept_calc(float set, float get, pid_t *pid);
extern void pid_antiwindup_calc(float set, float get, pid_t *pid);
extern void pid_tinter_calc(float set, float get, pid_t *pid);
extern void pid_incompletediff_calc(float set, float get, pid_t *pid);

#endif


#include "pid.h"
#include "main.h"
#include "math_formula.h"
#include "stdlib.h"
//云台PID
pid_t GM6020_PIDpos;
pid_t GM6020_PIDspd;
//底盘PID
pid_t World_Chassis_X_PIDpos;//世界坐标系下平移X位置
pid_t World_Chassis_Y_PIDpos;//世界坐标系下平移Y位置
pid_t Machine_Beta_PIDpos;//机器坐标系下自旋位置
pid_t M3508_PIDspd[3];//3508电机速度PID
pid_t M3508_PIDpos[3];//3508电机位置PID

/**************************************************************
	@brief:		pid参数初始化
	@param:		
	@retval: 		
	@supplement:	
**************************************************************/
void pid_init(pid_t *pid, uint8_t mode, float KP, float KI, float KD, float IMaxOut, float MaxOut)
{
	pid->mode = mode;
	pid->kp = KP;
	pid->ki = KI;
	pid->kd = KD;
	
	pid->Integrater_limitation = IMaxOut;
	pid->Output_limitation = MaxOut;
}

/**************************************************************
	@brief:		位置式pid
	@param:		
	@retval: 		
	@supplement:	
**************************************************************/
void pid_calc(float set, float get, pid_t *pid)
{
	pid->err[Now] = set - get;
	
	pid->pout = pid->kp * pid->err[Now];
	pid->iout+= pid->ki * pid->err[Now];
	pid->dout = pid->kd * (pid->err[Now] - pid->err[Last]);
	
	limit_amplitude(&pid->iout, &pid->Integrater_limitation);
	
	pid->out = pid->pout + pid->iout + pid->dout;
	
	limit_amplitude(&pid->out, &pid->Output_limitation);
	
	pid->err[Last] = pid->err[Now];
}

/**************************************************************
	@brief:		前馈环节
	@param:		T为1，J为1，前馈环节s+1，
	@retval: 		
	@supplement:	
**************************************************************/
float Forwardfeed_Value(pid_t *pid, float input, float dt)
{
	float forwardfeed_value;
	forwardfeed_value = input  +  (input - pid->last_input) / dt;
	pid->last_input = input;
	return forwardfeed_value;
}
/**************************************************************
	@brief:		积分分离pid
	@param:		
	@retval: 		
	@supplement:	Beta_Generation()函数选择积分添加区间
**************************************************************/
uint16_t Beta_Generation(float err, float epsilon)
{
	uint16_t beta = 0;
	if(abs(err)>epsilon)
		beta = 0;
	if(abs(err)<=epsilon)
		beta = 1;
	return beta;
}
void pid_isept_calc(float set, float get, pid_t *pid)
{
	pid->err[Now] = set - get;
	
	pid->pout = pid->kp * pid->err[Now];
	pid->dout = pid->kd * (pid->err[Now] - pid->err[Last]);
	
	uint16_t Beta = Beta_Generation(pid->err[Now], pid->epsilon);
	
	if(Beta)
	{
		pid->iout+= pid->ki * pid->err[Now];
		limit_amplitude(&pid->iout, &pid->Integrater_limitation);
		pid->out = pid->pout + pid->iout + pid->dout;
	}
	else
	{
		pid->out = pid->pout + pid->dout;
	}
	limit_amplitude(&pid->out, &pid->Output_limitation);
	
	pid->err[Last] = pid->err[Now];
}

/**************************************************************
	@brief:		抗积分饱和pid
	@param:		
	@retval: 		
	@supplement:	
**************************************************************/
void pid_antiwindup_calc(float set, float get, pid_t *pid)
{
	pid->err[Now] = set - get;
	
	if(pid->out > pid->maximum)
	{
		if(pid->err[Now]<=0)
			pid->iout+= pid->ki * pid->err[Now];
	}
	if(pid->out < pid->minimum)
	{
		if(pid->err[Now]>0)
			pid->iout+= pid->ki * pid->err[Now];
	}
	else
	{
		pid->iout+= pid->ki * pid->err[Now];
	}
	pid->pout = pid->kp * pid->err[Now];
	pid->dout = pid->kd * (pid->err[Now] - pid->err[Last]);
	
	pid->out = pid->pout + pid->iout + pid->dout;
	
	limit_amplitude(&pid->out, &pid->Output_limitation);
	
	pid->err[Last] = pid->err[Now];
	
}

/**************************************************************
	@brief:		梯形积分pid
	@param:		
	@retval: 		
	@supplement:	
**************************************************************/
void pid_tinter_calc(float set, float get, pid_t *pid)
{
	pid->err[Now] = set - get;
	
	pid->pout = pid->kp * pid->err[Now];
	pid->iout+= 0.5f * pid->ki * (pid->err[Now]+pid->err[Last]);
	pid->dout = pid->kd * (pid->err[Now] - pid->err[Last]);
	
	limit_amplitude(&pid->iout, &pid->Integrater_limitation);
	
	pid->out = pid->pout + pid->iout + pid->dout;
	
	limit_amplitude(&pid->out, &pid->Output_limitation);
	
	pid->err[Last] = pid->err[Now];
}

/**************************************************************
	@brief:		不完全微分pid
	@param:		
	@retval: 		
	@supplement:	
**************************************************************/
void pid_incompletediff_calc(float set, float get, pid_t *pid)
{
	pid->err[Now] = set - get;
	
	pid->pout = pid->kp * pid->err[Now];
	pid->iout+= pid->ki * pid->err[Now];
	pid->dout = pid->kd * (1-pid->alpha)*(pid->err[Now]-pid->err[Last]) + pid->alpha * pid->dout_last;
	
	limit_amplitude(&pid->iout, &pid->Integrater_limitation);
	
	pid->out = pid->pout + pid->iout + pid->dout;
	
	limit_amplitude(&pid->out, &pid->Output_limitation);
	
	pid->err[Last] = pid->err[Now];
	
	pid->dout_last = pid->dout;
}

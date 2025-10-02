#include "Path_Tracking.h"
#include "math.h"
#include "math_formula.h"

Trapazoid_t t_plan[4] = {0.0f};
Point_Parameter Path_Para[250];
point_t point;
Chassis_State Chassis;

float intergral_x,intergral_y;//积分项
float dy_t,dx_t;
float dx_last,dy_last;

volatile uint8_t Equalfraction = 0; 
volatile float last_tim = 0.001f,tim = 0.0f,tim_dete = 0.0f, tim_now, tim_pre_last;
//volatile float tim = 0.0f;
volatile float next_pointx, start_pointx, next_pointy, start_pointy;
volatile uint8_t direction_flag;
/**************************************************************
	@brief:		梯形速度规划
	@param:		
	@retval: 		
	@supplement:	
**************************************************************/
void trapezoid_speed_init(float xsum, float vm, float arise, float adown)
{
	float trise = 0.0f;
	float tdown = 0.0f;
//	float t_residue = 0.0f;
	float x_residue = 0.0f;
	float x_count = 0.0f;
	float vmax = 0.0f;
	trise = vm/arise;
	tdown = vm/adown;
	x_count = vm*(trise+tdown)/2;
	if(x_count > xsum)
	{
		vmax = sqrt(2*xsum*arise*adown)/(arise+adown);
		trise = vmax/arise;
		tdown = vmax/adown;
	}
	else
	{
		x_residue = xsum - x_count;
//		t_residue = x_residue/vm;
		vmax = vm;
	}
	t_plan[0].position = 0;                              t_plan[0].speed = 0;
	t_plan[1].position = 0.5f*trise*vmax;                t_plan[0].speed = vmax;
	t_plan[2].position = 0.5f*trise*vmax +  x_residue;   t_plan[0].speed = vmax;
	t_plan[3].position = xsum;                           t_plan[0].speed = 170;
}
//规划每一个阶段的速度
float trapezoid_vel(float real_area)
{
	float ti_speed;
	float ti_ratio = 0.0f;
	
	if((real_area>t_plan[0].position) && (real_area<t_plan[1].position))
	{
		ti_ratio = (real_area-t_plan[0].position)/((t_plan[1].position) - (t_plan[0].position));
		ti_speed = t_plan[0].speed + ti_ratio*(t_plan[1].speed - t_plan[0].speed);
	}
	else if((real_area>t_plan[1].position) && (real_area<t_plan[2].position))
	{
		ti_speed = t_plan[1].speed;
	}
	else
	{
		ti_ratio = (real_area-t_plan[2].position)/((t_plan[3].position) - (t_plan[2].position));
		ti_speed = t_plan[2].speed + ti_ratio*(t_plan[3].speed - t_plan[2].speed);
	}
	return ti_speed;
}

/**************************************************************
	@brief:		pid轨迹跟踪
	@param:		
	@retval: 		
	@supplement:	
**************************************************************/
void pid_track(float pre_x,float pre_y,float xt,float yt,float now_x,float now_y,float p,float p_t,float i,float d)
{
	float dx,dy;
	float dx_t,dy_t;
	float pid_x,pid_y;
	static float limitdata = 100.0f;
	//预瞄点-当前点
	dx = pre_x-now_x;
	dy = pre_y-now_y;
	//目标点-当前点
	dx_t = xt-now_x;
	dy_t = yt-now_y;
	//积分累积
	intergral_x=intergral_x+dx_t;
	intergral_y=intergral_y+dy_t;
	
	//对积分进行限幅,需要用100的指针
	limit_amplitude(&intergral_x, &limitdata);
	limit_amplitude(&intergral_y, &limitdata);
	
	pid_x = p * dx + p_t * dx_t + i * intergral_x + d * (dx_t - dx_last);
	pid_y = p * dy + p_t * dy_t + i * intergral_y + d * (dy_t - dy_last);
	
	dx_last = dx_t;
	dy_last = dy_t;
	
	Chassis.dir=RAD2ANGLE(atan2(pid_y,pid_x));
	if(tim_now < 0.3f)
	{
		Chassis.dir=RAD2ANGLE(atan2((Path_Para[1].y-Path_Para[0].y),(Path_Para[1].x-Path_Para[0].x) ));	
	}
	Chassis.dir=floor(Chassis.dir);
}

/**************************************************************
	@brief:		线性插值法
	@param:		
	@retval: 		
	@supplement:	
**************************************************************/
float linear_interpolation(Point_Parameter Path_Para[],float realx,float realy,int chan_flag)
{
	float insert_num = 0.0f;
	int last_tim_floor = (int)floor(last_tim);
	
	for(int i = last_tim_floor; i < Equalfraction - 1; i++)
	{
		if(chan_flag==2)
		{
			if((realy >= Path_Para[i].y && realy <= Path_Para[i+1].y) || (realy <= Path_Para[i].y && realy >= Path_Para[i+1].y))
			{
				insert_num = i + fabs(realy - Path_Para[i].y) / fabs(Path_Para[i+1].y - Path_Para[i].y);
				break;
			}
		}
		else
		{
			if((realx >= Path_Para[i].x && realx <= Path_Para[i+1].x) || (realx <= Path_Para[i].x && realx >= Path_Para[i+1].x))
			{
				insert_num = i + fabs(realx - Path_Para[i].x) / fabs(Path_Para[i+1].x - Path_Para[i].x);
				break;
			}
		}
	}
	return insert_num;
}
/**************************************************************
	@brief:		计算下一目标速度
	@param:		
	@retval: 		
	@supplement:	
**************************************************************/
void pid_prospect(float realx,float realy,float realv,int k)
{
	int tim_now_floor;
	float tim_now;
	//预瞄距离计算
	float area_factor = 0.005f * (k * realv);
	//判断初始点位相对零点位和下一个点位相对原点位的趋势是否一致
	if(tim_now < 0.1f)
	{
		realx = ((next_pointx - start_pointx) * (realx - start_pointx) > 0.0f)? realx:start_pointx;
	  realy = ((next_pointy - start_pointy) * (realy - start_pointy) > 0.0f)? realy:start_pointy;
	}
	
	float tim_test = last_tim;
	if(tim_test > (Equalfraction-1))  tim_test=Equalfraction-1;
	
	int tim_test_ceil=(int)ceil(tim_test);//向上取整
	int tim_test_floor=(int)floor(tim_test);//向下取整
	//direction_flag      1           2 
	//                x方向插值   y方向插值
	//以下判断语句主要选择区间更大的区域进行插值，可以更好规避因获取数据错误导致插值出错的情况
	if(fabs(Path_Para[tim_test_ceil].x - Path_Para[tim_test_floor].x) >= fabs(Path_Para[tim_test_ceil].y - Path_Para[tim_test_floor].y))
		direction_flag = 1;
	else
		direction_flag = 2;
	
	tim_now = linear_interpolation(Path_Para, realx, realy, direction_flag);//获得实际插值点
	
	//防止插值减少，导致出错，对插值点进行纠偏
	if(tim_now <= last_tim)
	{
		tim_now = last_tim + 0.001f;
	}
	else if(tim_now - last_tim >= 1.3f)
	{
		tim_now = last_tim + 1.3f;
	}
	else if(tim_now >= Equalfraction-1)
	{
		tim_now = Equalfraction-1;
	}
	
	tim_now_floor = (int)floor(tim_now);//对当前插值点向下取整
	last_tim = tim_now;
	//按一次线性比例插值,更新坐标
	point.update_x = Path_Para[tim_now_floor].x + (tim_now - tim_now_floor) * (Path_Para[(int)ceil(tim_now)].x - Path_Para[tim_now_floor].x);
	point.update_y = Path_Para[tim_now_floor].y + (tim_now - tim_now_floor) * (Path_Para[(int)ceil(tim_now)].y - Path_Para[tim_now_floor].y);
	//更新底盘位置的目标速度
	Chassis.vel = Path_Para[tim_now_floor].tgvel + (tim_now - tim_now_floor) * (Path_Para[(int)ceil(tim_now)].tgvel - Path_Para[tim_now_floor].tgvel);
	if(tim_now >= Equalfraction - 1 - 0.3f) {Chassis.vel = Path_Para[Equalfraction-1].tgvel;}
	
//tim是总路径的比例位置	
	tim =  tim_now + (area_factor) * (Equalfraction - 1);
	
	if(tim < tim_pre_last) tim = tim + 0.001f;
	if(tim < tim_pre_last + 2.0f) tim = tim_pre_last + 2.0f;
	if(tim >= Equalfraction - 1.0f) tim = Equalfraction - 1.0f;
	
//	tim_floor = floor(tim);
	
	if((tim < Equalfraction-1.0f-0.1f) && (tim >= 0.0f))
	{
		point.target_x = Path_Para[(int)floor(tim)].x + (tim - (int)floor(tim)) * (Path_Para[(int)ceil(tim)].x - Path_Para[(int)floor(tim)].x);
		point.target_y = Path_Para[(int)floor(tim)].y + (tim - (int)floor(tim)) * (Path_Para[(int)ceil(tim)].y - Path_Para[(int)floor(tim)].y);
	}
	else
	{
		point.target_x = Path_Para[Equalfraction-1].x;
		point.target_y = Path_Para[Equalfraction-1].y;
	}
	tim_pre_last = tim;
}

//void pid_beizer_speed(void)
//{
//	pid_prospect();
//	pid_track()
//}


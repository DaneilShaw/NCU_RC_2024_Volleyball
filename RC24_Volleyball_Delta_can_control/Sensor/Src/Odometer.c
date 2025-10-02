#include "Odometer.h"

#define r 1

void Odom_Coordinate_Calc(Odom_Coordinate * Odom, float omniwheel_vel1, float omniwheel_vel2, float omniwheel_vel3, float period)
{
	Odom->vx = - omniwheel_vel1 + (omniwheel_vel2 + omniwheel_vel3) / 2.0f;
	Odom->x = Odom->x + (Odom->vx + Odom->last_vx) * period / 2;
	Odom->last_vx = Odom->vx;
	
	Odom->vy = (omniwheel_vel3 - omniwheel_vel2) * 1.732f / 2;
	Odom->y = Odom->y + (Odom->vy + Odom->last_vy) * period / 2;
	Odom->last_vy = Odom->vy;
	
	Odom->w = (omniwheel_vel1 + omniwheel_vel2 + omniwheel_vel3) / r;
	Odom->omega = Odom->omega + (Odom->w + Odom->last_w) * period / 2;
	Odom->last_w = Odom->w;
}



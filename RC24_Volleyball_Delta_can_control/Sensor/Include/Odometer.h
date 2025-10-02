#ifndef __ODOMETER_H
#define __ODOMETER_H

typedef struct
{
	float x;
	float y;
	float omega;
	
	float vx;
	float last_vx;
	float vy;
	float last_vy;
	float w;
	float last_w;
}Odom_Coordinate;


#endif


#ifndef __VISION_RECEIVE_H
#define __VISION_RECEIVE_H
#include "My_Includes.h"

typedef struct
{
	float X_axis;
	float Y_axis;
	float X_Spd;
	float Y_Spd;
}Camera_Coordinate_t;

#define VisionRx_Size 32


extern Camera_Coordinate_t Camera_Coordinate;

extern uint8_t VisionRx_DataBuf[VisionRx_Size];

extern void VisionRecv_USART2_IRQHandler(void);

#endif


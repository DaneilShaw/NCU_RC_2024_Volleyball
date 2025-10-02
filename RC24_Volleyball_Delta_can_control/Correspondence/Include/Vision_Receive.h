#ifndef __VISION_RECEIVE_H
#define __VISION_RECEIVE_H
#include "My_Includes.h"
#include "AppRemote_Task.h"



#define VisionRx_Size 32
#define VisionTx_Size 32

typedef struct
{
    float X_axis;
    float Y_axis;
}Camera_Coordinate_t;


extern uint8_t VisionTx_DataBuf[VisionTx_Size];
extern uint8_t VisionRx_DataBuf[VisionRx_Size];
extern Camera_Coordinate_t Camera_Coordinate;

extern void VisionRecv_USART2_IRQHandler(void);

extern void Vision_Decode(volatile const uint8_t *buff, Camera_Coordinate_t *Camera_Coordinate);

#endif


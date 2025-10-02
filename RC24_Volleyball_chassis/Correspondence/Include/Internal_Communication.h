#ifndef __INTERNAL_COMMUNICATION_H
#define __INTERNAL_COMMUNICATION_H

#include "My_Includes.h"
#include "usart.h"
#include "bsp_usart.h"
#include "AppRemote_Task.h"

#define TransferData_Size 9

extern uint16_t TransferData_RxLen;
extern uint8_t Transfer_RxData[TransferData_Size];

extern void Communicate_USART3_IRQHandler(void);

extern void Data_transfer(uint8_t RxData, int RxData_Size);

#endif


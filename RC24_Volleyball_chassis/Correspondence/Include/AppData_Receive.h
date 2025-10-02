#ifndef __APPDATA_RECEIVE_H
#define __APPDATA_RECEIVE_H
#include "My_Includes.h"

enum
{
	txarm_finish,//发送完成
	txarm_ing,//正在发送
};

#define AppRx_Size 8
#define app_uart huart7
#define dma_app_rx hdma_uart7_rx
#define DMA_AppUART_Buf DMA_UART7_RxBuf
#define DMA_AppUART_RX_SIZE DMA_UART7_RX_SIZE

extern uint16_t APP_RxLen;
extern uint8_t AppRx_DataBuf[AppRx_Size];

extern void AppRecv_UART7_IRQHandler(void);


#endif


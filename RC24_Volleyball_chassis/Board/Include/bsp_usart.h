#ifndef __BSP_USART_H
#define __BSP_USART_H
#include "struct_typedef.h"
#include "usart.h"
#include <string.h>

#define DMA_USART6_RX_SIZE 60
#define DMA_USART3_RX_SIZE 9
#define DMA_UART8_RX_SIZE 127
#define DMA_UART7_RX_SIZE 9
#define DMA_USART2_RX_SIZE 33

extern uint8_t DMA_USART2_RxBuf[DMA_USART2_RX_SIZE];
extern uint8_t DMA_USART6_RxBuf[DMA_USART6_RX_SIZE];
extern uint8_t DMA_USART3_RxBuf[DMA_USART3_RX_SIZE];
extern uint8_t DMA_UART8_RxBuf[DMA_UART8_RX_SIZE];
extern uint8_t DMA_UART7_RxBuf[DMA_UART7_RX_SIZE];

HAL_StatusTypeDef Bsp_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
void DMA_IDLE_IT_Init(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
void UART_PC_IT_Init(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
extern void uart8_tx_dma_enable(uint8_t *data, uint16_t len);

#endif


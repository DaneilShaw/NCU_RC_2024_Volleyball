#include "bsp.h"

void BSP_Init()
{
	TIM_Motor_Init(&htim1);//TIM1初始化
	DMA_IDLE_IT_Init(&huart6, DMA_USART6_RxBuf, DMA_USART6_RX_SIZE);//USART6的DMA初始化
	DMA_IDLE_IT_Init(&huart3, DMA_USART3_RxBuf, DMA_USART3_RX_SIZE);//USART3的DMA初始化
	DMA_IDLE_IT_Init(&huart8, DMA_UART8_RxBuf, DMA_UART8_RX_SIZE);//UART8的DMA初始化
	DMA_IDLE_IT_Init(&huart7, DMA_UART7_RxBuf, DMA_UART7_RX_SIZE);//UART7的DMA初始化
	DMA_IDLE_IT_Init(&huart2, DMA_USART2_RxBuf, DMA_USART2_RX_SIZE);//USART2的DMA初始化
	can_filter_init();//CAN1初始化
}


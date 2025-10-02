#include "bsp.h"

void BSP_Init()
{
	TIM_Motor_Init(&htim1);//TIM1��ʼ��
	DMA_IDLE_IT_Init(&huart6, DMA_USART6_RxBuf, DMA_USART6_RX_SIZE);//USART6��DMA��ʼ��
	DMA_IDLE_IT_Init(&huart3, DMA_USART3_RxBuf, DMA_USART3_RX_SIZE);//USART3��DMA��ʼ��
	DMA_IDLE_IT_Init(&huart8, DMA_UART8_RxBuf, DMA_UART8_RX_SIZE);//UART8��DMA��ʼ��
	DMA_IDLE_IT_Init(&huart7, DMA_UART7_RxBuf, DMA_UART7_RX_SIZE);//UART7��DMA��ʼ��
	DMA_IDLE_IT_Init(&huart2, DMA_USART2_RxBuf, DMA_USART2_RX_SIZE);//USART2��DMA��ʼ��
	can_filter_init();//CAN1��ʼ��
}


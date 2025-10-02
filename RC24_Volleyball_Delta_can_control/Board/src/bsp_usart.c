#include "bsp_usart.h"

uint8_t DMA_USART2_RxBuf[DMA_USART2_RX_SIZE];
uint8_t DMA_USART6_RxBuf[DMA_USART6_RX_SIZE];
uint8_t DMA_USART3_RxBuf[DMA_USART3_RX_SIZE];
uint8_t DMA_UART8_RxBuf[DMA_UART8_RX_SIZE];
uint8_t DMA_UART7_RxBuf[DMA_UART7_RX_SIZE];

void DMA_IDLE_IT_Init(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
	/*利用DMA接受数据，不加此句接收不到第一次传进来的实数据，
	是空的，且此时接收到的数据长度为缓存器的数据长度*/
	HAL_UART_Receive_DMA(huart,pData,Size);
	/*开启串口空闲中断*/
	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
}

void uart8_tx_dma_enable(uint8_t *data, uint16_t len)
{
  //等待上一次数据发送完毕
	while(HAL_DMA_GetState(&hdma_uart8_tx) != HAL_DMA_STATE_READY);
	
	//关闭DMA
  __HAL_DMA_DISABLE(&hdma_uart8_tx);

  //clear flag
  //清除标志位
  __HAL_DMA_CLEAR_FLAG(&hdma_uart8_tx, DMA_HISR_TCIF7);
  __HAL_DMA_CLEAR_FLAG(&hdma_uart8_tx, DMA_HISR_HTIF7);

  HAL_UART_Transmit_DMA(&huart8, data, len);
}




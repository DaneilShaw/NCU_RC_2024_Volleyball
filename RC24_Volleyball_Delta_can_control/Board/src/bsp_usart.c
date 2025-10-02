#include "bsp_usart.h"

uint8_t DMA_USART2_RxBuf[DMA_USART2_RX_SIZE];
uint8_t DMA_USART6_RxBuf[DMA_USART6_RX_SIZE];
uint8_t DMA_USART3_RxBuf[DMA_USART3_RX_SIZE];
uint8_t DMA_UART8_RxBuf[DMA_UART8_RX_SIZE];
uint8_t DMA_UART7_RxBuf[DMA_UART7_RX_SIZE];

void DMA_IDLE_IT_Init(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
	/*����DMA�������ݣ����Ӵ˾���ղ�����һ�δ�������ʵ���ݣ�
	�ǿյģ��Ҵ�ʱ���յ������ݳ���Ϊ�����������ݳ���*/
	HAL_UART_Receive_DMA(huart,pData,Size);
	/*�������ڿ����ж�*/
	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
}

void uart8_tx_dma_enable(uint8_t *data, uint16_t len)
{
  //�ȴ���һ�����ݷ������
	while(HAL_DMA_GetState(&hdma_uart8_tx) != HAL_DMA_STATE_READY);
	
	//�ر�DMA
  __HAL_DMA_DISABLE(&hdma_uart8_tx);

  //clear flag
  //�����־λ
  __HAL_DMA_CLEAR_FLAG(&hdma_uart8_tx, DMA_HISR_TCIF7);
  __HAL_DMA_CLEAR_FLAG(&hdma_uart8_tx, DMA_HISR_HTIF7);

  HAL_UART_Transmit_DMA(&huart8, data, len);
}




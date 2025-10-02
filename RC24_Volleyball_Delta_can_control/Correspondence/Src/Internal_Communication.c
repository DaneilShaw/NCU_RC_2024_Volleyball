#include "Internal_Communication.h"

uint16_t TransferData_RxLen;
uint8_t Transfer_RxData[TransferData_Size];
//ע���ж��Ƿ���Ժ��ڲ�ͨ��ͬʱ���ã����߶���������֪ͨ����
void Communicate_USART3_IRQHandler(void)
{
	BaseType_t pxHigherPriorityTaskWoken;

	if(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE))
	{
		/*��������жϱ�־*/
 		__HAL_UART_CLEAR_IDLEFLAG(&huart3);
		/*�ر�DMA����*/
		__HAL_DMA_DISABLE(&hdma_usart3_rx);
		
		TransferData_RxLen = DMA_USART3_RX_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);
		if(TransferData_RxLen >= 8)
		{
			/*��ȡ��������*/
			memcpy(Transfer_RxData, DMA_USART3_RxBuf, 8);//�ӻ�������ȡ��������	
		}
		/*�������*/
		memset(DMA_USART3_RxBuf, 0, DMA_USART3_RX_SIZE);
		
		/*�����趨���������ݳ���*/
		hdma_usart3_rx.Instance->NDTR = DMA_USART3_RX_SIZE;
		
		/*ʹ��DMA*/
		__HAL_DMA_ENABLE(&hdma_usart3_rx);
		
		/*���Ӧ�����������֪ͨ*/
		vTaskNotifyGiveFromISR(AppRemote_Handle, &pxHigherPriorityTaskWoken);
		
		/*�����Ҫ�Ļ�������һ�������л���ϵͳ�ж��Ƿ���Ҫ�����л�*/
		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
	}
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart3)
	{
		__HAL_DMA_CLEAR_FLAG(&hdma_usart3_tx, DMA_HISR_TCIF7); //���DMA2_Steam7������ɱ�־
		task_flag.appdata_txarm_flag = txarm_finish;
		HAL_UART_DMAStop(&huart3);		//��������Ժ�رմ���DMA,ȱ����һ�������
	}
}

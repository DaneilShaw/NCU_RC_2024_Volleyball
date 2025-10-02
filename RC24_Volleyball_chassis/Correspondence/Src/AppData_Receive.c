#include "AppData_Receive.h"

uint16_t APP_RxLen;
uint8_t AppRx_DataBuf[AppRx_Size];

void AppRecv_UART7_IRQHandler(void)
{
	BaseType_t pxHigherPriorityTaskWoken;
	//�ж��Ƿ�Ϊ�����ж�
	if(__HAL_UART_GET_FLAG(&app_uart, UART_FLAG_IDLE))
	{
		/*��������жϱ�־*/
		__HAL_UART_CLEAR_IDLEFLAG(&app_uart);
		/*�ر�DMA����*/
		__HAL_DMA_DISABLE(&dma_app_rx);
		/*��ȡ���յ���GO������ݳ����뷢�͵��Ƿ�һ��*/
		APP_RxLen = DMA_AppUART_RX_SIZE - __HAL_DMA_GET_COUNTER(&dma_app_rx);
		if(APP_RxLen >= 8)
		{
			/*��ʼ���յ����������,��ʱ�����͵����������*/
			/*��ȡ��������*/
			memcpy(AppRx_DataBuf, DMA_AppUART_Buf, 8);//�ӻ�������ȡ��������		
		}
		/*��������������*/
		memset(DMA_AppUART_Buf, 0, DMA_AppUART_RX_SIZE);
		/*�����趨���������ݳ���*/
		hdma_uart7_rx.Instance->NDTR = DMA_AppUART_RX_SIZE;
		/*ʹ��DMA*/
		__HAL_DMA_ENABLE(&dma_app_rx);
		/*���Ӧ�����������֪ͨ*/
    vTaskNotifyGiveFromISR(AppRemote_Handle, &pxHigherPriorityTaskWoken);
	  /*�����Ҫ�Ļ�������һ�������л���ϵͳ�ж��Ƿ���Ҫ�����л�*/
		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken); 
	}
}




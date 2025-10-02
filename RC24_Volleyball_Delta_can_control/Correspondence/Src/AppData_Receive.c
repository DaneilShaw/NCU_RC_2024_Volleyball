#include "AppData_Receive.h"

uint16_t APP_RxLen;
uint8_t AppRx_DataBuf[AppRx_Size];
uint8_t AppTx_DataBuf[AppTx_Size];
//ע���ж��Ƿ���Ժ����²�ͨ��ͬʱ���ã����߶���������֪ͨ����
void AppRecv_UART7_IRQHandler(void)
{
	/*BaseType_t ��FreeRTOs�е�һ���������ͣ�ͨ����������ֵ
	pxHigherPriorityTaskWoken�����������ָʾ�Ƿ��и������ȼ������������֪ͨ��������
	���pxHigherPriorityTaskWoken������ΪpdTRUE�����и������ȼ������񱻻���*/
	BaseType_t pxHigherPriorityTaskWoken;
	
	/*�ж��Ƿ�Ϊ�����жϣ��������ֵΪ0�����ʾUART���ڴ�������
	����ֵ��0��Ϊ�棩����UART���У������Դ�������
	UART_FLAG_IDLE��־�����ã����ʾUART���ڿ���״̬*/
	if(__HAL_UART_GET_FLAG(&app_uart, UART_FLAG_IDLE))
	{
		/*��������жϱ�־����UART_FLAG_IDLE ��־������ʱ����ʾUART�Լ���������ݽ��գ���û�и�����������ڽ���
		��Ҫ��UART�ٴν����µ�����ǰ���������־λ����*/
		__HAL_UART_CLEAR_IDLEFLAG(&app_uart);
		
		/*�ر�DMA���䣬��UART������ݽ��պ���Ӧ��DMA����ҲӦ��ֹͣ*/
		__HAL_DMA_DISABLE(&dma_app_rx);
		
		/*��ȡ���յ������ݳ����뷢�͵��Ƿ�һ��
		�ѽ��ܵ������� = DMA�������ܻ����С - ʣ��δ���յ��ֽ���*/
		APP_RxLen = DMA_AppUART_RX_SIZE - __HAL_DMA_GET_COUNTER(&dma_app_rx);
		if(APP_RxLen >= 8)
		{
			/*��ʼ���շ�������,��ʱ�����Ϳ�������*/
			/*��ȡ��������,��DMA�е����� ���Ƶ�AppRx_DataBuf��*/
			memcpy(AppRx_DataBuf, DMA_AppUART_Buf, 8);//�ӻ�������ȡ��������		
		}
		
		/*��DMA��������������*/
		memset(DMA_AppUART_Buf, 0, DMA_AppUART_RX_SIZE);
		
		/*�����趨���������ݳ���*/
		hdma_uart7_rx.Instance->NDTR = DMA_AppUART_RX_SIZE;
		
		/*ʹ��DMA*/
		__HAL_DMA_ENABLE(&dma_app_rx);
		
		/*���Ӧ�����������֪ͨ
		vTaskNotifyGiveFromISR ��������ָ����������һ��֪ͨ*/
    vTaskNotifyGiveFromISR(AppRemote_Handle, &pxHigherPriorityTaskWoken);
		
	  /*�����Ҫ�Ļ�������һ�������л���ϵͳ�ж��Ƿ���Ҫ�����л�*/
		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken); 
	}
}




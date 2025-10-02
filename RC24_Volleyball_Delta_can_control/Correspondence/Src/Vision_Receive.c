#include "Vision_Receive.h"

uint16_t Vision_RxLen;
uint8_t VisionRx_DataBuf[VisionRx_Size];
uint8_t VisionTx_DataBuf[VisionTx_Size];

Camera_Coordinate_t Camera_Coordinate;

void Vision_Decode(volatile const uint8_t *buff, Camera_Coordinate_t *Camera_Coordinate);

void VisionRecv_USART2_IRQHandler(void)
{
//	BaseType_t pxHigherPriorityTaskWoken;
	//�ж��Ƿ�Ϊ�����ж�
	if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE))
	{
		/*��������жϱ�־*/
		__HAL_UART_CLEAR_IDLEFLAG(&huart2);
		
		/*�ر�DMA����*/
		__HAL_DMA_DISABLE(&hdma_usart2_rx);
		
		/*��ȡ���յ���GO������ݳ����뷢�͵��Ƿ�һ��*/
		Vision_RxLen = DMA_USART2_RX_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
		if(Vision_RxLen == 3)
		{
			/*��ʼ���յ����������,��ʱ�����͵����������*/
			/*��ȡ��������*/
			memcpy(VisionRx_DataBuf, DMA_USART2_RxBuf, 3);//�ӻ�������ȡ��������	
		}
		if(Vision_RxLen >= 8)
		{
			/*��ʼ���յ����������,��ʱ�����͵����������*/
			/*��ȡ��������*/
			memcpy(VisionRx_DataBuf, DMA_USART2_RxBuf, 32);//�ӻ�������ȡ��������	

			/*�Ӿ����ݽ��봦��*/			
			Vision_Decode(VisionRx_DataBuf, &Camera_Coordinate);			
		}
		
		/*��������������*/
		memset(DMA_USART2_RxBuf, 0, DMA_USART2_RX_SIZE);
		
		/*�����趨���������ݳ���*/
		hdma_usart2_rx.Instance->NDTR = DMA_USART2_RX_SIZE;
		
		/*ʹ��DMA*/
		__HAL_DMA_ENABLE(&hdma_usart2_rx);
		
//		/*�������Ҫ�õ� Vision_Task �������Ӿ����ݣ���ע�͵�����֪ͨ*/
//		/*���Ӧ�����������֪ͨ*/
//    vTaskNotifyGiveFromISR(HitBall_Handle, &pxHigherPriorityTaskWoken);
//		
//		/*�����Ҫ�Ļ�������һ�������л���ϵͳ�ж��Ƿ���Ҫ�����л�*/
//		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
	}
}

/**
  * @brief				�Ӿ����ݽ��뺯��
  * @param[in]			buff:ָ�����ݽ��������ָ�룬�������ָ�� VisionRx_DataBuf ��Transfer_RxData �ȵ�
	* @param[out]		
  * @retval				
*/
void Vision_Decode(volatile const uint8_t *buff, Camera_Coordinate_t *Camera_Coordinate)
{
    int symbol,i;
    if(buff[2] == 'X')
    {
        if(buff[3] == '0') symbol = 1;
        else if(buff[3] == '-') symbol = -1;
        i = 4;
        while(buff[i] == '0') {i++;}
        Camera_Coordinate->X_axis = symbol * (atoi((const char *)&buff[i]));//atoi ���ַ���ת��Ϊ����
    }
    if(buff[9] == 'Y')
    {
        if(buff[10] == '0') symbol = 1;
        else if(buff[10] == '-') symbol = -1;
        i = 11;
        while(buff[i] == '0') {i++;}
        Camera_Coordinate->Y_axis = symbol * (atoi((const char *)&buff[i]));
    }

}



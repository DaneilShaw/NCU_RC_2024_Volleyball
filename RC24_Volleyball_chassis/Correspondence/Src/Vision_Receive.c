#include "Vision_Receive.h"

Camera_Coordinate_t Camera_Coordinate;

uint16_t Vision_RxLen;
uint8_t VisionRx_DataBuf[VisionRx_Size];

void Vision_Decode(void);

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
 		if(Vision_RxLen == VisionRx_Size)
		{
			/*��ʼ���յ����������,��ʱ�����͵����������*/
			/*��ȡ��������*/
			memcpy(VisionRx_DataBuf, DMA_USART2_RxBuf, VisionRx_Size);//�ӻ�������ȡ��������		 
			/*�������ݽ���*/
			Vision_Decode();
		}
		if(Vision_RxLen == 3)
		{
			/*��ʼ���յ����������,��ʱ�����͵����������*/
			/*��ȡ��������*/
			memcpy(VisionRx_DataBuf, DMA_USART2_RxBuf, VisionRx_Size);//�ӻ�������ȡ��������		 
		}
		/*��������������*/
		memset(DMA_USART2_RxBuf, 0, DMA_USART2_RX_SIZE);
		/*�����趨���������ݳ���*/
		hdma_usart2_rx.Instance->NDTR = DMA_USART2_RX_SIZE;
		/*ʹ��DMA*/
		__HAL_DMA_ENABLE(&hdma_usart2_rx);
	}
}

/**************************************************************
	@brief:  �Ӿ����뺯��
	@param:		������Ӿ�������λ�����ݺ��ٶ�����
	@retval: 		
	@supplement: 				
**************************************************************/
void Vision_Decode(void)
{
	int symbol,i;
	/******************λ�����ݽ���********************/
	/*X�����λ�����ݽ���*/
	if(VisionRx_DataBuf[2] == 'X')
	{
		if(VisionRx_DataBuf[3] == '0') symbol = 1;
		else if(VisionRx_DataBuf[3] == '-') symbol = -1;
		i = 4;
		while(VisionRx_DataBuf[i] == '0') {i++;}
		Camera_Coordinate.X_axis = symbol * (atoi((const char *)&VisionRx_DataBuf[i]));
		/*atoi()���������һ���ַ���������һ������*/
	}
	
	/*Y�����λ�����ݽ���*/
	if(VisionRx_DataBuf[9] == 'Y')
	{
		if(VisionRx_DataBuf[10] == '0') symbol = 1;
		else if(VisionRx_DataBuf[10] == '-') symbol = -1;
		i = 11;
		while(VisionRx_DataBuf[i] == '0') {i++;}
		Camera_Coordinate.Y_axis = symbol * (atoi((const char *)&VisionRx_DataBuf[i]));
	}
	
	/******************�ٶ����ݽ���********************/
	/*X�����ٶ����ݽ���*/
	if(VisionRx_DataBuf[16] == 'X')
	{
		if(VisionRx_DataBuf[17] == '0') symbol = 1;
		else if(VisionRx_DataBuf[17] == '-') symbol = -1;
		i = 18;
		while(VisionRx_DataBuf[i] == '0') {i++;}
		Camera_Coordinate.X_Spd = atoi((const char *)&VisionRx_DataBuf[i]);
	}
	
	/*Y�����ٶ����ݽ���*/
	if(VisionRx_DataBuf[23] == 'Y')
	{
		if(VisionRx_DataBuf[24] == '0') symbol = 1;
		else if(VisionRx_DataBuf[24] == '-') symbol = -1;
		i = 25;
		while(VisionRx_DataBuf[i] == '0') {i++;}
		Camera_Coordinate.Y_Spd = atoi((const char *)&VisionRx_DataBuf[i]);
	}
}


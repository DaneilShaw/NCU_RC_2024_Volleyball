
#include "Scope_printf.h"

int8_t pw_correct;
uint8_t PW_RxLen;
uint8_t name[];//ʾ����������������

char PwRx_DataBuf[30];//ʾ�������ݽ���
char cmd[4];//ʾ�������ִ洢
float WaveRx_Kp,WaveRx_Ki,WaveRx_Kd;//ʾ�����޸�ID

void Usart_SendArray(UART_HandleTypeDef *huart, uint8_t *array, uint16_t num);
void Usart_SendByte( UART_HandleTypeDef *huart,uint8_t ch );   //���Ͳ�������

/**************************************************************
	@brief:		printf����
	@param:		
	@retval: 		
	@supplement:	���ڴ�ӡ����ĸ�������
	              usart_printf("��ʽ���ַ��� %d %s", integer_variable, string_variable);
**************************************************************/
void usart_printf(const char *fmt,...)
{
 static uint8_t tx_buf[256] = {0};//��̬�����������ڴ����ʽ������ַ���
 static va_list ap;//����һ���䳤�����б�
 static uint16_t len;//���������ڴ����ʽ�����ַ����ĳ���
 va_start(ap, fmt);//��ʼ���䳤�����б�fmt�����һ����������
 //return length of string 
 //�����ַ�������
 len = vsprintf((char *)tx_buf, fmt, ap);
 va_end(ap);//����䳤�����б�
 //����UART8 TX DMA�������䴢����tx_buf�г���Ϊ'len'�ĸ�ʽ���ַ���
 uart8_tx_dma_enable(tx_buf, len);
}

/**************************************************************
	@brief:		send_wave����
	@param:		nameΪ�����������ƣ�
	@retval: 		
	@supplement:	ʾ��������
**************************************************************/
void send_wave(uint8_t *name, uint8_t name_len, float *channels_t, uint16_t channel_len) 
{
    //����ͨ����֡ͷ֡β
    uint8_t frameNameHead[] = "AABBCC";
    uint8_t frameNameEnd[] = "CCBBAA";
    
    //��������֡ͷ֡β
    uint8_t frameDataHead[] = "DDEEFF";
    uint8_t frameDataEnd[] = "FFEEDD";
    
    //��ֵ����
//	float channels[7];//ʾ������������ֵ
//  channels[0] = 100.0f;
//	channels[1] = 200.0f;
//	channels[2] = 300.0f;
//	channels[3] = 400.0f;
//	channels[4] = 500.0f;
//	channels[5] = 600.0f;
//	channels[6] = 700.0f;
	
    //ͨ������1������λ����������
    //����sizeof(frameNameHead)-1) �еļ�1�����Ǻ��Ե�frameNameHead�ַ������� ��\0�� �ַ�
    HAL_UART_Transmit(&huart8, frameNameHead, sizeof(frameNameHead) - 1, 100); 
    HAL_UART_Transmit(&huart8, name, name_len - 1, 100);
    HAL_UART_Transmit(&huart8, frameNameEnd, sizeof(frameNameEnd) - 1, 100);
    
    HAL_UART_Transmit(&huart8, frameDataHead, sizeof(frameDataHead) - 1, 100);
    Usart_SendArray(&huart8, (uint8_t *)channels_t, channel_len);
    HAL_UART_Transmit(&huart8, frameDataEnd, sizeof(frameDataEnd) - 1, 100);
}

void Usart_SendArray(UART_HandleTypeDef *huart, uint8_t *array, uint16_t num)
{
	uint8_t i;
	for(i=0; i<num; i++)
	{
		Usart_SendByte(huart,array[i]);
	}
}

void Usart_SendByte( UART_HandleTypeDef *huart,uint8_t ch )   //���Ͳ�������
{
  HAL_UART_Transmit(huart, (uint8_t *)&ch, 1, 10); 
}


/**************************************************************
	@brief:		uart8_wave_getdata����
	@param:		
	@retval: 		
	@supplement:	��λ���������ݽ���
**************************************************************/
void uart8_wave_getdata(void)
{
	sscanf(PwRx_DataBuf, "%3s=%f,%f,%f", cmd, &WaveRx_Kp, &WaveRx_Ki, &WaveRx_Kd);
	memset(PwRx_DataBuf, 0, sizeof(PwRx_DataBuf));
}

void WaveRecv_UART8_IRQHandler(void)
{
	//�ж��Ƿ�Ϊ�����ж�
	if(__HAL_UART_GET_FLAG(&pw_uart, UART_FLAG_IDLE))
	{
		/*��������жϱ�־*/
		__HAL_UART_CLEAR_IDLEFLAG(&pw_uart);
		/*�ر�DMA����*/
		__HAL_DMA_DISABLE(&dma_pw_rx);
		/*��ȡ���յ���GO������ݳ����뷢�͵��Ƿ�һ��*/
		PW_RxLen = DMA_UART8_RX_SIZE - __HAL_DMA_GET_COUNTER(&dma_pw_rx);
		if(DMA_UART8_RxBuf[PW_RxLen-1] == '#')
		{
			pw_correct = 1;
			/*��ʼ���յ����������,��ʱ�����͵����������*/
			/*��ȡ��������*/
			memcpy(PwRx_DataBuf, DMA_UART8_RxBuf, PW_RxLen-1);//�ӻ�������ȡ��������	
      uart8_wave_getdata();			
		}
		else
			pw_correct = -1;
		/*��������������*/
		memset(DMA_UART8_RxBuf, 0, DMA_UART8_RX_SIZE);
		/*�����趨���������ݳ���*/
		hdma_uart8_rx.Instance->NDTR = DMA_UART8_RX_SIZE;
		/*ʹ��DMA*/
		__HAL_DMA_ENABLE(&dma_pw_rx);
	}
}


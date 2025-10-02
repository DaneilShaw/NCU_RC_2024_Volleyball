
#include "Scope_printf.h"

int8_t pw_correct;
uint8_t PW_RxLen;
uint8_t name[];//示波器发送数据名称

char PwRx_DataBuf[30];//示波器数据接收
char cmd[4];//示波器名字存储
float WaveRx_Kp,WaveRx_Ki,WaveRx_Kd;//示波器修改ID

void Usart_SendArray(UART_HandleTypeDef *huart, uint8_t *array, uint16_t num);
void Usart_SendByte( UART_HandleTypeDef *huart,uint8_t ch );   //发送参数数据

/**************************************************************
	@brief:		printf函数
	@param:		
	@retval: 		
	@supplement:	用于打印电机的各项数据
	              usart_printf("格式化字符串 %d %s", integer_variable, string_variable);
**************************************************************/
void usart_printf(const char *fmt,...)
{
 static uint8_t tx_buf[256] = {0};//静态缓冲区，用于储存格式化后的字符串
 static va_list ap;//声明一个变长参数列表
 static uint16_t len;//变量，用于储存格式化后字符串的长度
 va_start(ap, fmt);//初始化变长参数列表，fmt是最后一个命名参数
 //return length of string 
 //返回字符串长度
 len = vsprintf((char *)tx_buf, fmt, ap);
 va_end(ap);//清零变长参数列表
 //启用UART8 TX DMA，并传输储存在tx_buf中长度为'len'的格式化字符串
 uart8_tx_dma_enable(tx_buf, len);
}

/**************************************************************
	@brief:		send_wave函数
	@param:		name为发送数据名称，
	@retval: 		
	@supplement:	示波器函数
**************************************************************/
void send_wave(uint8_t *name, uint8_t name_len, float *channels_t, uint16_t channel_len) 
{
    //定义通道名帧头帧尾
    uint8_t frameNameHead[] = "AABBCC";
    uint8_t frameNameEnd[] = "CCBBAA";
    
    //定义数据帧头帧尾
    uint8_t frameDataHead[] = "DDEEFF";
    uint8_t frameDataEnd[] = "FFEEDD";
    
    //赋值数据
//	float channels[7];//示波器发送数据值
//  channels[0] = 100.0f;
//	channels[1] = 200.0f;
//	channels[2] = 300.0f;
//	channels[3] = 400.0f;
//	channels[4] = 500.0f;
//	channels[5] = 600.0f;
//	channels[6] = 700.0f;
	
    //通过串口1，向上位机发送数据
    //下面sizeof(frameNameHead)-1) 中的减1，就是忽略掉frameNameHead字符串最后的 ‘\0’ 字符
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

void Usart_SendByte( UART_HandleTypeDef *huart,uint8_t ch )   //发送参数数据
{
  HAL_UART_Transmit(huart, (uint8_t *)&ch, 1, 10); 
}


/**************************************************************
	@brief:		uart8_wave_getdata函数
	@param:		
	@retval: 		
	@supplement:	上位机发送数据解码
**************************************************************/
void uart8_wave_getdata(void)
{
	sscanf(PwRx_DataBuf, "%3s=%f,%f,%f", cmd, &WaveRx_Kp, &WaveRx_Ki, &WaveRx_Kd);
	memset(PwRx_DataBuf, 0, sizeof(PwRx_DataBuf));
}

void WaveRecv_UART8_IRQHandler(void)
{
	//判断是否为空闲中断
	if(__HAL_UART_GET_FLAG(&pw_uart, UART_FLAG_IDLE))
	{
		/*清除接收中断标志*/
		__HAL_UART_CLEAR_IDLEFLAG(&pw_uart);
		/*关闭DMA传输*/
		__HAL_DMA_DISABLE(&dma_pw_rx);
		/*获取接收到的GO电机数据长度与发送的是否一致*/
		PW_RxLen = DMA_UART8_RX_SIZE - __HAL_DMA_GET_COUNTER(&dma_pw_rx);
		if(DMA_UART8_RxBuf[PW_RxLen-1] == '#')
		{
			pw_correct = 1;
			/*开始接收电机反馈数据,此时不发送电机控制数据*/
			/*获取反馈数据*/
			memcpy(PwRx_DataBuf, DMA_UART8_RxBuf, PW_RxLen-1);//从缓存区获取反馈数据	
      uart8_wave_getdata();			
		}
		else
			pw_correct = -1;
		/*缓存区数据清零*/
		memset(DMA_UART8_RxBuf, 0, DMA_UART8_RX_SIZE);
		/*重新设定缓存区数据长度*/
		hdma_uart8_rx.Instance->NDTR = DMA_UART8_RX_SIZE;
		/*使能DMA*/
		__HAL_DMA_ENABLE(&dma_pw_rx);
	}
}


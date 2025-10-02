#include "Vision_Receive.h"

Camera_Coordinate_t Camera_Coordinate;

uint16_t Vision_RxLen;
uint8_t VisionRx_DataBuf[VisionRx_Size];

void Vision_Decode(void);

void VisionRecv_USART2_IRQHandler(void)
{
//	BaseType_t pxHigherPriorityTaskWoken;
	//判断是否为空闲中断
	if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE))
	{
		/*清除接收中断标志*/
		__HAL_UART_CLEAR_IDLEFLAG(&huart2);
		/*关闭DMA传输*/
		__HAL_DMA_DISABLE(&hdma_usart2_rx);
		/*获取接收到的GO电机数据长度与发送的是否一致*/
		Vision_RxLen = DMA_USART2_RX_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
 		if(Vision_RxLen == VisionRx_Size)
		{
			/*开始接收电机反馈数据,此时不发送电机控制数据*/
			/*获取反馈数据*/
			memcpy(VisionRx_DataBuf, DMA_USART2_RxBuf, VisionRx_Size);//从缓存区获取反馈数据		 
			/*进行数据解码*/
			Vision_Decode();
		}
		if(Vision_RxLen == 3)
		{
			/*开始接收电机反馈数据,此时不发送电机控制数据*/
			/*获取反馈数据*/
			memcpy(VisionRx_DataBuf, DMA_USART2_RxBuf, VisionRx_Size);//从缓存区获取反馈数据		 
		}
		/*缓存区数据清零*/
		memset(DMA_USART2_RxBuf, 0, DMA_USART2_RX_SIZE);
		/*重新设定缓存区数据长度*/
		hdma_usart2_rx.Instance->NDTR = DMA_USART2_RX_SIZE;
		/*使能DMA*/
		__HAL_DMA_ENABLE(&hdma_usart2_rx);
	}
}

/**************************************************************
	@brief:  视觉解码函数
	@param:		解算出视觉反馈的位置数据和速度数据
	@retval: 		
	@supplement: 				
**************************************************************/
void Vision_Decode(void)
{
	int symbol,i;
	/******************位置数据解码********************/
	/*X方向的位置数据解码*/
	if(VisionRx_DataBuf[2] == 'X')
	{
		if(VisionRx_DataBuf[3] == '0') symbol = 1;
		else if(VisionRx_DataBuf[3] == '-') symbol = -1;
		i = 4;
		while(VisionRx_DataBuf[i] == '0') {i++;}
		Camera_Coordinate.X_axis = symbol * (atoi((const char *)&VisionRx_DataBuf[i]));
		/*atoi()函数会解析一个字符串并返回一个整数*/
	}
	
	/*Y方向的位置数据解码*/
	if(VisionRx_DataBuf[9] == 'Y')
	{
		if(VisionRx_DataBuf[10] == '0') symbol = 1;
		else if(VisionRx_DataBuf[10] == '-') symbol = -1;
		i = 11;
		while(VisionRx_DataBuf[i] == '0') {i++;}
		Camera_Coordinate.Y_axis = symbol * (atoi((const char *)&VisionRx_DataBuf[i]));
	}
	
	/******************速度数据解码********************/
	/*X方向速度数据解码*/
	if(VisionRx_DataBuf[16] == 'X')
	{
		if(VisionRx_DataBuf[17] == '0') symbol = 1;
		else if(VisionRx_DataBuf[17] == '-') symbol = -1;
		i = 18;
		while(VisionRx_DataBuf[i] == '0') {i++;}
		Camera_Coordinate.X_Spd = atoi((const char *)&VisionRx_DataBuf[i]);
	}
	
	/*Y方向速度数据解码*/
	if(VisionRx_DataBuf[23] == 'Y')
	{
		if(VisionRx_DataBuf[24] == '0') symbol = 1;
		else if(VisionRx_DataBuf[24] == '-') symbol = -1;
		i = 25;
		while(VisionRx_DataBuf[i] == '0') {i++;}
		Camera_Coordinate.Y_Spd = atoi((const char *)&VisionRx_DataBuf[i]);
	}
}


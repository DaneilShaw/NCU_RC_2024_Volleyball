#include "Vision_Receive.h"

uint16_t Vision_RxLen;
uint8_t VisionRx_DataBuf[VisionRx_Size];
uint8_t VisionTx_DataBuf[VisionTx_Size];

Camera_Coordinate_t Camera_Coordinate;

void Vision_Decode(volatile const uint8_t *buff, Camera_Coordinate_t *Camera_Coordinate);

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
		if(Vision_RxLen == 3)
		{
			/*开始接收电机反馈数据,此时不发送电机控制数据*/
			/*获取反馈数据*/
			memcpy(VisionRx_DataBuf, DMA_USART2_RxBuf, 3);//从缓存区获取反馈数据	
		}
		if(Vision_RxLen >= 8)
		{
			/*开始接收电机反馈数据,此时不发送电机控制数据*/
			/*获取反馈数据*/
			memcpy(VisionRx_DataBuf, DMA_USART2_RxBuf, 32);//从缓存区获取反馈数据	

			/*视觉数据解码处理*/			
			Vision_Decode(VisionRx_DataBuf, &Camera_Coordinate);			
		}
		
		/*缓存区数据清零*/
		memset(DMA_USART2_RxBuf, 0, DMA_USART2_RX_SIZE);
		
		/*重新设定缓存区数据长度*/
		hdma_usart2_rx.Instance->NDTR = DMA_USART2_RX_SIZE;
		
		/*使能DMA*/
		__HAL_DMA_ENABLE(&hdma_usart2_rx);
		
//		/*如果不需要用到 Vision_Task 来处理视觉数据，即注释掉任务通知*/
//		/*向对应句柄的任务发送通知*/
//    vTaskNotifyGiveFromISR(HitBall_Handle, &pxHigherPriorityTaskWoken);
//		
//		/*如果需要的话，进行一次任务切换，系统判断是否需要进行切换*/
//		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
	}
}

/**
  * @brief				视觉数据解码函数
  * @param[in]			buff:指向数据接收数组的指针，例如可以指向 VisionRx_DataBuf ，Transfer_RxData 等等
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
        Camera_Coordinate->X_axis = symbol * (atoi((const char *)&buff[i]));//atoi 将字符串转换为整型
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



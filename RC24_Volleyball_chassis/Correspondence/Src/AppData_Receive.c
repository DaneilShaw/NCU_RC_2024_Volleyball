#include "AppData_Receive.h"

uint16_t APP_RxLen;
uint8_t AppRx_DataBuf[AppRx_Size];

void AppRecv_UART7_IRQHandler(void)
{
	BaseType_t pxHigherPriorityTaskWoken;
	//判断是否为空闲中断
	if(__HAL_UART_GET_FLAG(&app_uart, UART_FLAG_IDLE))
	{
		/*清除接收中断标志*/
		__HAL_UART_CLEAR_IDLEFLAG(&app_uart);
		/*关闭DMA传输*/
		__HAL_DMA_DISABLE(&dma_app_rx);
		/*获取接收到的GO电机数据长度与发送的是否一致*/
		APP_RxLen = DMA_AppUART_RX_SIZE - __HAL_DMA_GET_COUNTER(&dma_app_rx);
		if(APP_RxLen >= 8)
		{
			/*开始接收电机反馈数据,此时不发送电机控制数据*/
			/*获取反馈数据*/
			memcpy(AppRx_DataBuf, DMA_AppUART_Buf, 8);//从缓存区获取反馈数据		
		}
		/*缓存区数据清零*/
		memset(DMA_AppUART_Buf, 0, DMA_AppUART_RX_SIZE);
		/*重新设定缓存区数据长度*/
		hdma_uart7_rx.Instance->NDTR = DMA_AppUART_RX_SIZE;
		/*使能DMA*/
		__HAL_DMA_ENABLE(&dma_app_rx);
		/*向对应句柄的任务发送通知*/
    vTaskNotifyGiveFromISR(AppRemote_Handle, &pxHigherPriorityTaskWoken);
	  /*如果需要的话，进行一次任务切换，系统判断是否需要进行切换*/
		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken); 
	}
}




#include "AppData_Receive.h"

uint16_t APP_RxLen;
uint8_t AppRx_DataBuf[AppRx_Size];
uint8_t AppTx_DataBuf[AppTx_Size];
//注意判断是否可以和上下层通信同时调用，两者都包含任务通知功能
void AppRecv_UART7_IRQHandler(void)
{
	/*BaseType_t 是FreeRTOs中的一个数据类型，通常用作布尔值
	pxHigherPriorityTaskWoken这个变量用于指示是否有更高优先级的任务因这次通知而被唤醒
	如果pxHigherPriorityTaskWoken被设置为pdTRUE，则有更高优先级的任务被唤醒*/
	BaseType_t pxHigherPriorityTaskWoken;
	
	/*判断是否为空闲中断，如果返回值为0，则表示UART正在传输数据
	返回值非0（为真），则UART空闲，即可以传输数据
	UART_FLAG_IDLE标志被设置，则表示UART处于空闲状态*/
	if(__HAL_UART_GET_FLAG(&app_uart, UART_FLAG_IDLE))
	{
		/*清除接收中断标志，当UART_FLAG_IDLE 标志被设置时，表示UART以及完成了数据接收，并没有更多的数据正在接收
		需要在UART再次接收新的数据前，将这个标志位清零*/
		__HAL_UART_CLEAR_IDLEFLAG(&app_uart);
		
		/*关闭DMA传输，当UART完成数据接收后，相应的DMA传输也应该停止*/
		__HAL_DMA_DISABLE(&dma_app_rx);
		
		/*获取接收到的数据长度与发送的是否一致
		已接受到的数据 = DMA接收区总缓存大小 - 剩余未接收的字节数*/
		APP_RxLen = DMA_AppUART_RX_SIZE - __HAL_DMA_GET_COUNTER(&dma_app_rx);
		if(APP_RxLen >= 8)
		{
			/*开始接收反馈数据,此时不发送控制数据*/
			/*获取反馈数据,将DMA中的数据 复制到AppRx_DataBuf中*/
			memcpy(AppRx_DataBuf, DMA_AppUART_Buf, 8);//从缓存区获取反馈数据		
		}
		
		/*将DMA缓存区数据清零*/
		memset(DMA_AppUART_Buf, 0, DMA_AppUART_RX_SIZE);
		
		/*重新设定缓存区数据长度*/
		hdma_uart7_rx.Instance->NDTR = DMA_AppUART_RX_SIZE;
		
		/*使能DMA*/
		__HAL_DMA_ENABLE(&dma_app_rx);
		
		/*向对应句柄的任务发送通知
		vTaskNotifyGiveFromISR 函数是向指定的任务发送一个通知*/
    vTaskNotifyGiveFromISR(AppRemote_Handle, &pxHigherPriorityTaskWoken);
		
	  /*如果需要的话，进行一次任务切换，系统判断是否需要进行切换*/
		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken); 
	}
}




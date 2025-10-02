#include "Internal_Communication.h"

uint16_t TransferData_RxLen;
uint8_t Transfer_RxData[TransferData_Size];

/***********************底盘主控和上层主控通过串口3 相互通信*************************/
void Communicate_USART3_IRQHandler(void)
{
//	BaseType_t pxHigherPriorityTaskWoken;

	if(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE))
	{
		/*清除接收中断标志*/
		__HAL_UART_CLEAR_IDLEFLAG(&huart3);
		/*关闭DMA传输*/
		__HAL_DMA_DISABLE(&hdma_usart3_rx);
		
		TransferData_RxLen = DMA_USART3_RX_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);
		if(TransferData_RxLen == 8)
		{
			/*获取反馈数据*/
			memcpy(Transfer_RxData, DMA_USART3_RxBuf, 8);//从缓存区获取反馈数据	
		}
		/*清空数据*/
		memset(DMA_USART3_RxBuf, 0, DMA_USART3_RX_SIZE);
		/*重新设定缓存区数据长度*/
		hdma_usart3_rx.Instance->NDTR = DMA_USART3_RX_SIZE;
		/*使能DMA*/
		__HAL_DMA_ENABLE(&hdma_usart3_rx);
//		/*向对应句柄的任务发送通知*/
//		vTaskNotifyGiveFromISR(AppRemote_Handle, &pxHigherPriorityTaskWoken);
//		/*如果需要的话，进行一次任务切换，系统判断是否需要进行切换*/
//		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
	}
}

/**************************************************************
	@brief:		
	@param:		
	@retval: 		
	@supplement:	这个函数在UART（通用异步收发传输器）发送操作完成时被调用
**************************************************************/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart3)
	{
	/*huart3 发送完成后执行的代码*/
		__HAL_DMA_CLEAR_FLAG(&hdma_usart3_tx, DMA_HISR_TCIF7); //清除传输完成标志
		task_flag.appdata_txarm_flag = txarm_finish;	//将传输标志赋值
		HAL_UART_DMAStop(&huart3);		//传输完成以后关闭串口DMA,缺了这一句会死机
	}
}



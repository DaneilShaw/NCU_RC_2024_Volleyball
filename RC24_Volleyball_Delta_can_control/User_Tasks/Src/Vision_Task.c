#include "Vision_Task.h"

int8_t vision_flag = 0;

//void Vision_Task(void const * argument)
//{
//	uint32_t VisionNotifyValue = pdFALSE;
//	
//	TickType_t xLastWakeTime;
//	xLastWakeTime = xTaskGetTickCount();//获取时钟节拍
//	for(;;)
//	{
//	VisionNotifyValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
//	if(VisionNotifyValue == 1)
//	{
//		
//		task_flag.visiondata_txarm_flag = txarm_ing;
//		HAL_UART_Transmit_DMA(&huart3, VisionRx_DataBuf, sizeof(VisionRx_DataBuf));
//		while(task_flag.visiondata_txarm_flag != txarm_finish){};//DMA发送指令为非阻塞发送，此处等待DMA发送完成，再执行接下来的任务

//		/*缓存区数据清零*/
//		memset(VisionRx_DataBuf, 0, 24);
//		}
//		osDelayUntil(&xLastWakeTime, 4);//绝对延时,其它注意不要出现偶数延时，否则其它任务无法获取CPU权限
//	}
//}


#include "Vision_Task.h"

int8_t vision_flag = 0;

//void Vision_Task(void const * argument)
//{
//	uint32_t VisionNotifyValue = pdFALSE;
//	
//	TickType_t xLastWakeTime;
//	xLastWakeTime = xTaskGetTickCount();//��ȡʱ�ӽ���
//	for(;;)
//	{
//	VisionNotifyValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
//	if(VisionNotifyValue == 1)
//	{
//		
//		task_flag.visiondata_txarm_flag = txarm_ing;
//		HAL_UART_Transmit_DMA(&huart3, VisionRx_DataBuf, sizeof(VisionRx_DataBuf));
//		while(task_flag.visiondata_txarm_flag != txarm_finish){};//DMA����ָ��Ϊ���������ͣ��˴��ȴ�DMA������ɣ���ִ�н�����������

//		/*��������������*/
//		memset(VisionRx_DataBuf, 0, 24);
//		}
//		osDelayUntil(&xLastWakeTime, 4);//������ʱ,����ע�ⲻҪ����ż����ʱ���������������޷���ȡCPUȨ��
//	}
//}


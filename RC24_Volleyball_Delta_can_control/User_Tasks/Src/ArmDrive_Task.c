#include "ArmDrive_Task.h"

/*****************ȫ�ֱ���*****************/
extern SemaphoreHandle_t ArmDrive_xSemaphore;
extern void Get_GO1_Arm_Basepos(void);

void ArmDrive_Task(void const * argument)
{
	/*����GO1�����ʼλ��*/
	Get_GO1_Arm_Basepos();
	for(;;)
	{
//		time = HAL_GetTick() - last_time;
		if(xSemaphoreTake(ArmDrive_xSemaphore, portMAX_DELAY) == pdTRUE)//�ź������գ�������ʱ��Ӱ�쵽�������ݽ���
		{
			modify_data(&Go_TailArm_send);
			if((ulTaskNotifyTake(pdTRUE, 2)) != 1){};

//			last_time = HAL_GetTick();
		}
	}
}


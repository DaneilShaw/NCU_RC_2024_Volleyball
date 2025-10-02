#include "ArmDrive_Task.h"

/*****************全局变量*****************/
extern SemaphoreHandle_t ArmDrive_xSemaphore;
extern void Get_GO1_Arm_Basepos(void);

void ArmDrive_Task(void const * argument)
{
	/*宇树GO1电机初始位置*/
	Get_GO1_Arm_Basepos();
	for(;;)
	{
//		time = HAL_GetTick() - last_time;
		if(xSemaphoreTake(ArmDrive_xSemaphore, portMAX_DELAY) == pdTRUE)//信号量接收，绝对延时会影响到总线数据接收
		{
			modify_data(&Go_TailArm_send);
			if((ulTaskNotifyTake(pdTRUE, 2)) != 1){};

//			last_time = HAL_GetTick();
		}
	}
}


#include "AppRemote_Task.h"

float L ; 
Task_Flag task_flag;
int Light_On = 0 ;

void AppRemote_Task(void const * argument)
{
	uint32_t AppNotifyValue = pdFALSE;
	
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();//获取时钟节拍
	for(;;)
	{
		/*调用ulTaskNotifyTake函数来接收任务通知*/
		AppNotifyValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		if(AppNotifyValue == 1)
		{
			memcpy(AppRx_DataBuf, Transfer_RxData, 8);
	    /********************************************机械臂********************************************/
			
			/***************************************机械臂复位***************************************/
		  if(AppRx_DataBuf[0] == 'B' && AppRx_DataBuf[1] == 'A' && AppRx_DataBuf[6] == 'L' && AppRx_DataBuf[7] == 'L' )
		  {
			  if(AppRx_DataBuf[2] == '0' && AppRx_DataBuf[3] == '1' && AppRx_DataBuf[4] == '0' && AppRx_DataBuf[5] == '0' )
			  {
					/*将程序复位，标志位归0*/
					task_flag.cushionball_flag  = 0;
					task_flag.hitball_flag = 0 ;
					Light_On = 0 ; 
					/*3508电流发送设置为0*/
					M3508_PID_spd[0].out = 0;
					M3508_PID_spd[1].out = 0;
					M3508_PID_spd[2].out = 0;
					M3508_PID_spd[3].out = 0;
			  }
				/***************************************开始垫球***************************************/
			  if(AppRx_DataBuf[2] == '0' && AppRx_DataBuf[3] == '1' && AppRx_DataBuf[4] == '0' && AppRx_DataBuf[5] == '1' )
			  {
					/*将球垫起*/
					task_flag.cushionball_flag = Cushion_To_Vision;
					/*将发球的标志位设置为0*/
					task_flag.hitball_flag = 0 ;
			  }
				/***************************************开始发球***************************************/
				if(AppRx_DataBuf[2] == '0' && AppRx_DataBuf[3] == '1' && AppRx_DataBuf[4] == '0' && AppRx_DataBuf[5] == '2' )
			  {
					/*将球高高垫起*/
					task_flag.hitball_flag = Serve_To_Vision;
					/*将垫球的标志位设置为0*/
					task_flag.cushionball_flag  = 0;
					Light_On = 0 ; 
					/*发球程序启动的标志位*/
			  }
		  }
			/********************************************底盘********************************************/
			if(AppRx_DataBuf[0] == 'V' && AppRx_DataBuf[1] == 'S' && AppRx_DataBuf[6] == 'N' && AppRx_DataBuf[7] == 'T' )
		  {
				if(AppRx_DataBuf[3] != '4' && AppRx_DataBuf[3] != '9')
				{
					Chassis_World.Alpha = atoi((const char *)&AppRx_DataBuf[3]);
				}
				if(AppRx_DataBuf[3] == '9'&& AppRx_DataBuf[4] == '9' && AppRx_DataBuf[5] == '9')
				{
					Chassis_World.Alpha = 0;
				}
		  }
		  memset(AppRx_DataBuf, 0, 8);
		}
		osDelayUntil(&xLastWakeTime, 3);//绝对延时
	}
}




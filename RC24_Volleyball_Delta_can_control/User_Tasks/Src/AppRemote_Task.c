#include "AppRemote_Task.h"

float L ; 
Task_Flag task_flag;
int Light_On = 0 ;

void AppRemote_Task(void const * argument)
{
	uint32_t AppNotifyValue = pdFALSE;
	
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();//��ȡʱ�ӽ���
	for(;;)
	{
		/*����ulTaskNotifyTake��������������֪ͨ*/
		AppNotifyValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		if(AppNotifyValue == 1)
		{
			memcpy(AppRx_DataBuf, Transfer_RxData, 8);
	    /********************************************��е��********************************************/
			
			/***************************************��е�۸�λ***************************************/
		  if(AppRx_DataBuf[0] == 'B' && AppRx_DataBuf[1] == 'A' && AppRx_DataBuf[6] == 'L' && AppRx_DataBuf[7] == 'L' )
		  {
			  if(AppRx_DataBuf[2] == '0' && AppRx_DataBuf[3] == '1' && AppRx_DataBuf[4] == '0' && AppRx_DataBuf[5] == '0' )
			  {
					/*������λ����־λ��0*/
					task_flag.cushionball_flag  = 0;
					task_flag.hitball_flag = 0 ;
					Light_On = 0 ; 
					/*3508������������Ϊ0*/
					M3508_PID_spd[0].out = 0;
					M3508_PID_spd[1].out = 0;
					M3508_PID_spd[2].out = 0;
					M3508_PID_spd[3].out = 0;
			  }
				/***************************************��ʼ����***************************************/
			  if(AppRx_DataBuf[2] == '0' && AppRx_DataBuf[3] == '1' && AppRx_DataBuf[4] == '0' && AppRx_DataBuf[5] == '1' )
			  {
					/*�������*/
					task_flag.cushionball_flag = Cushion_To_Vision;
					/*������ı�־λ����Ϊ0*/
					task_flag.hitball_flag = 0 ;
			  }
				/***************************************��ʼ����***************************************/
				if(AppRx_DataBuf[2] == '0' && AppRx_DataBuf[3] == '1' && AppRx_DataBuf[4] == '0' && AppRx_DataBuf[5] == '2' )
			  {
					/*����߸ߵ���*/
					task_flag.hitball_flag = Serve_To_Vision;
					/*������ı�־λ����Ϊ0*/
					task_flag.cushionball_flag  = 0;
					Light_On = 0 ; 
					/*������������ı�־λ*/
			  }
		  }
			/********************************************����********************************************/
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
		osDelayUntil(&xLastWakeTime, 3);//������ʱ
	}
}




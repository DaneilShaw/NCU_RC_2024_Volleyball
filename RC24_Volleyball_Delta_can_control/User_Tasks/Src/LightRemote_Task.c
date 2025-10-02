#include "LightRemote_Task.h"
/**********************ȫ�ֱ���**********************/
//extern int data_point;
int test_flag=1;
uint8_t Light1, Light2, Light3;
uint8_t To_Chassis [3] = {"get"}; 
/**********************ȫ�ֺ���**********************/
void Light_sensor(void);

////char buff1[8]={'V','S','0','2','1','2','N','T'};0
////char buff2[8]={'V','S','0','0','9','2','N','T'};
////char buff3[8]={'V','S','0','3','3','5','N','T'};
void LightRemote_Task(void const * argument)
{
  TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();//��ȡʱ�ӽ�
	
  for(;;)
	{   
		if(Light_On == 1)
		{
     Light_sensor();
			if(Light1 == 1 || Light2 == 1 )
			{
				task_flag.cushionball_flag = up_action;//����������
				/*ͨ��APP����ָ��󣬱�־λ��Ϊ txarm_ing*/
				task_flag.appdata_txarm_flag = txarm_ing;
				HAL_UART_Transmit_DMA(&huart3, To_Chassis, sizeof(To_Chassis));	
				/*����ɴ���֮�󣬱�־λ�ḳֵΪtxarm_finish��ֱ����־λ��Ϊtxarm_finish���Ż������һ���ж�*/
				while(task_flag.appdata_txarm_flag != txarm_finish){};//DMA����ָ��Ϊ���������ͣ��˴��ȴ�DMA������ɣ���ִ�н�����������				
			}
		}
	osDelayUntil(&xLastWakeTime, 5);//������ʱ
	}
}

void Light_sensor(void)
{
	Light1 = Photo_Electricity_Read(GPIOE, GPIO_PIN_5);
	Light2 = Photo_Electricity_Read(GPIOE, GPIO_PIN_6);
}
    
//    for(;;){
//        
//        Light_sensor();
//        if(Light_1==0&&Light_2==0&&Light_3==0){
//            data_point = 5;//·���㸳��
//            task_flag.cushionball_flag = up_action;//����������
//                         }
//        else if(Light_1!= 0&&Light_2==0&&Light_3==0 ){
//                    memcpy(AppRx_DataBuf, buff1, 8);
//                    task_flag.appdata_txarm_flag = txarm_ing;
//					HAL_UART_Transmit_DMA(&huart3, AppRx_DataBuf, sizeof(AppRx_DataBuf));//����̴���ָ���������ݰ�
//					while(task_flag.appdata_txarm_flag != txarm_finish){};//DMA����ָ��Ϊ���������ͣ��˴��ȴ�DMA������ɣ���ִ�н�����������
//                          //��--���·����ƶ�����
//             }
//             if(Light_2!= 0&&Light_1==0&&Light_3==0 ){
//                    memcpy(AppRx_DataBuf, buff2, 8);
//                    task_flag.appdata_txarm_flag = txarm_ing;
//					HAL_UART_Transmit_DMA(&huart3, AppRx_DataBuf, sizeof(AppRx_DataBuf));
//					while(task_flag.appdata_txarm_flag != txarm_finish){};//DMA����ָ��Ϊ���������ͣ��˴��ȴ�DMA������ɣ���ִ�н�����������
//                          //��--���Ϸ����ƶ�����

//             }
//             if(Light_3!= 0&&Light_2==0&&Light_1==0 ){
//                    memcpy(AppRx_DataBuf, buff3, 8);
//                    task_flag.appdata_txarm_flag = txarm_ing;
//					HAL_UART_Transmit_DMA(&huart3, AppRx_DataBuf, sizeof(AppRx_DataBuf));
//					while(task_flag.appdata_txarm_flag != txarm_finish){};//DMA����ָ��Ϊ���������ͣ��˴��ȴ�DMA������ɣ���ִ�н�����������
//                          //��--���·����ƶ�����

//             }
//             osDelayUntil(&xLastWakeTime, 2);//������ʱ

//           }


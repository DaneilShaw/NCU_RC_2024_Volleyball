#include "LightRemote_Task.h"
/**********************全局变量**********************/
//extern int data_point;
int test_flag=1;
uint8_t Light1, Light2, Light3;
uint8_t To_Chassis [3] = {"get"}; 
/**********************全局函数**********************/
void Light_sensor(void);

////char buff1[8]={'V','S','0','2','1','2','N','T'};0
////char buff2[8]={'V','S','0','0','9','2','N','T'};
////char buff3[8]={'V','S','0','3','3','5','N','T'};
void LightRemote_Task(void const * argument)
{
  TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();//获取时钟节
	
  for(;;)
	{   
		if(Light_On == 1)
		{
     Light_sensor();
			if(Light1 == 1 || Light2 == 1 )
			{
				task_flag.cushionball_flag = up_action;//触发垫球动作
				/*通过APP发送指令后，标志位变为 txarm_ing*/
				task_flag.appdata_txarm_flag = txarm_ing;
				HAL_UART_Transmit_DMA(&huart3, To_Chassis, sizeof(To_Chassis));	
				/*在完成传输之后，标志位会赋值为txarm_finish，直到标志位变为txarm_finish，才会进行下一步判断*/
				while(task_flag.appdata_txarm_flag != txarm_finish){};//DMA发送指令为非阻塞发送，此处等待DMA发送完成，再执行接下来的任务				
			}
		}
	osDelayUntil(&xLastWakeTime, 5);//绝对延时
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
//            data_point = 5;//路径点赋零
//            task_flag.cushionball_flag = up_action;//触发垫球动作
//                         }
//        else if(Light_1!= 0&&Light_2==0&&Light_3==0 ){
//                    memcpy(AppRx_DataBuf, buff1, 8);
//                    task_flag.appdata_txarm_flag = txarm_ing;
//					HAL_UART_Transmit_DMA(&huart3, AppRx_DataBuf, sizeof(AppRx_DataBuf));//向底盘传输指定方向数据包
//					while(task_flag.appdata_txarm_flag != txarm_finish){};//DMA发送指令为非阻塞发送，此处等待DMA发送完成，再执行接下来的任务
//                          //向--左下方向移动底盘
//             }
//             if(Light_2!= 0&&Light_1==0&&Light_3==0 ){
//                    memcpy(AppRx_DataBuf, buff2, 8);
//                    task_flag.appdata_txarm_flag = txarm_ing;
//					HAL_UART_Transmit_DMA(&huart3, AppRx_DataBuf, sizeof(AppRx_DataBuf));
//					while(task_flag.appdata_txarm_flag != txarm_finish){};//DMA发送指令为非阻塞发送，此处等待DMA发送完成，再执行接下来的任务
//                          //向--正上方向移动底盘

//             }
//             if(Light_3!= 0&&Light_2==0&&Light_1==0 ){
//                    memcpy(AppRx_DataBuf, buff3, 8);
//                    task_flag.appdata_txarm_flag = txarm_ing;
//					HAL_UART_Transmit_DMA(&huart3, AppRx_DataBuf, sizeof(AppRx_DataBuf));
//					while(task_flag.appdata_txarm_flag != txarm_finish){};//DMA发送指令为非阻塞发送，此处等待DMA发送完成，再执行接下来的任务
//                          //向--右下方向移动底盘

//             }
//             osDelayUntil(&xLastWakeTime, 2);//绝对延时

//           }


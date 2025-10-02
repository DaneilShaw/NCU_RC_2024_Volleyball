#include "bsp_can.h"
#include "main.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

	/*CAN过滤器的配置与启动*/
void can_filter_init(void)
{
	/*定义一个名为can_filter_st的CAN_FilterTypeDef类型的变量*/
  CAN_FilterTypeDef can_filter_st;
	
	/*启动这个过滤器*/
  can_filter_st.FilterActivation = ENABLE;
	
	/*设置过滤器的工作模式为ID和掩码模式*/
  can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
	
	/*设置过滤器尺寸为32位，ID和掩码都为32位*/
  can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
	
	/*过滤器的ID高16位和低16位都设置为0，表示匹配所有ID的高16位和低16位*/
  can_filter_st.FilterIdHigh = 0x0000;
  can_filter_st.FilterIdLow = 0x0000;
	
	/*设置过滤器的掩码高16位和低16位都为0，表示不屏蔽任何ID位*/
  can_filter_st.FilterMaskIdHigh = 0x0000;
  can_filter_st.FilterMaskIdLow = 0x0000;
	
	/*选择使用CAN硬件的第0个过滤器*/
  can_filter_st.FilterBank = 0;
	
	/*将这个过滤器分配给CAN接收FIFO*/
  can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
	
	/*使用HAL库函数配置CAN1的过滤器*/
  HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
	
	/*启动CAN1控制器*/
  HAL_CAN_Start(&hcan1);
	
	/*激活CAN1的接收FIFO消息到达中断通知，当FIFO0接受到新的消息时，将触发一个中断*/
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

	
	/*以下为CAN2通信过滤器初始化配置*/
	
	/*当CAN控制器支持多个过滤器组时，这个成员指定了从哪个过滤器开始配置*/
  can_filter_st.SlaveStartFilterBank = 14;
	/*这个成员指定了当前正在配置的过滤器组的编号*/
  can_filter_st.FilterBank = 14;
	/*调用HAL库函数配置hcan2过滤器*/
  HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
	/*调用HAL库函数来启动hcan2 CAN控制器*/
  HAL_CAN_Start(&hcan2);
	/*激活hcan2的接收FIFO0消息到达终端通知*/
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

}


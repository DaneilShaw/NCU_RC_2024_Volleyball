#include "vesc6.h"
#include "string.h"

/*用于存放vesc的返回值*/
can_status_msg can_vescmsg[vesc6_number]={0};
can_status_msg2 can_vescmsg2[vesc6_number]={0};
can_status_msg3 can_vescmsg3[vesc6_number]={0};
can_status_msg4 can_vescmsg4[vesc6_number]={0};
can_status_msg5 can_vescmsg5[vesc6_number]={0};

/*实时的控制变量*/
volatile float vesc_current = 0.0f;
volatile float vesc_rpm = 0.0f;
volatile float vesc_pos = 0.0f;
volatile uint8_t vesc_mode=0;  //电调默认为速度模式


void comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len) 
{
  CAN_TxHeaderTypeDef TxMessage;
	uint32_t box;
	uint8_t Data[8];
	
  if (len > 8) 
	 {
    len = 8;                          //最多发送8个字节的数据
   } 
   
 TxMessage.ExtId=id;                  // 设置扩展ID（29位）
 TxMessage.IDE=CAN_ID_EXT;            // 扩展帧格式
 TxMessage.RTR=0;                     // 设置为数据帧
 TxMessage.DLC=len;                   // 传送的数据长度,字节数
 memcpy(Data, data, len);   // 将data地址的数据，拷贝到TxMessage.Data中，字节数为len

/******申请空闲消息盒子并提出发送申请******/
	HAL_CAN_AddTxMessage(&hcan2,&TxMessage,Data,&box);
/***************查询消息发送情况**************/
	while(HAL_CAN_AddTxMessage(&hcan2,&TxMessage,Data,&box) != HAL_OK);   //这里为方便调试，写成发送失败程序就卡死的形式
	
/******消息发送成功*****/
}

/**************************************************************
	@brief:		设置驱动器占空比
	@param:		
	@retval: 		
	@supplement:	
**************************************************************/
void vesc_can_set_duty(uint8_t controller_id, float duty)       
{
 int32_t send_index = 4;
 uint8_t buffer[4]={0};
 int32_t send_data2=0;
 send_data2=duty * 100000.0f;
 buffer[3]=(send_data2)&0xff;
 buffer[2]=(send_data2>>8)&0xff;
 buffer[1]=(send_data2>>16)&0xff;
 buffer[0]=(send_data2>>24);
 comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_DUTY << 8), 
buffer, send_index);//
}

/**************************************************************
	@brief:		设置驱动器转速
	@param:		单位：RPM
	@retval: 		
	@supplement:	
**************************************************************/
void vesc_can_set_rpm(uint8_t controller_id, float rpm)      
{
 int32_t send_index = 4;
 uint8_t buffer[4]={0};
 int32_t send_data2=0;
 send_data2=rpm*poles;    //将RPM转换成ERPM
 buffer[3]=(send_data2)&0xff;
 buffer[2]=(send_data2>>8)&0xff;
 buffer[1]=(send_data2>>16)&0xff;
 buffer[0]=(send_data2>>24);
 comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_RPM << 8), 
buffer, send_index);//
}

/**************************************************************
	@brief:		设置驱动器电流
	@param:		单位：A
	@retval: 		
	@supplement:	
**************************************************************/
void vesc_can_set_current(uint8_t controller_id, float current)     
{
 int32_t send_index = 4;
 uint8_t buffer[4]={0};
 int32_t send_data2=0;
 send_data2=current * 1000.0f;
 buffer[3]=(send_data2)&0xff;
 buffer[2]=(send_data2>>8)&0xff;
 buffer[1]=(send_data2>>16)&0xff;
 buffer[0]=(send_data2>>24);
 comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8), 
buffer, send_index);//
}

/**************************************************************
	@brief:		设置驱动器刹车电流
	@param:		单位：A
	@retval: 		
	@supplement:	用于主动刹车，受最大电流限制
**************************************************************/
void vesc_can_set_current_brake(uint8_t controller_id, float current) 
{
 int32_t send_index = 4;
 uint8_t buffer[4]={0};
 int32_t send_data2=0;
 send_data2=current * 1000.0f;
 buffer[3]=(send_data2)&0xff;
 buffer[2]=(send_data2>>8)&0xff;
 buffer[1]=(send_data2>>16)&0xff;
 buffer[0]=(send_data2>>24);
 comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), 
buffer, send_index);//
}

/**************************************************************
	@brief:		设置驱动器绝对位置
	@param:		单位：°
	@retval: 		
	@supplement:	上电位置为0°，运动方向为优弧方向（即：当前为0°，目标为350°时，电机会选择反向旋转10°）
**************************************************************/
void vesc_can_set_position(uint8_t controller_id, float position) 
{
 int32_t send_index = 4;
 uint8_t buffer[4]={0};
 int32_t send_data2=0;
 send_data2=position*1000000 ;
 buffer[3]=(send_data2)&0xff;
 buffer[2]=(send_data2>>8)&0xff;
 buffer[1]=(send_data2>>16)&0xff;
 buffer[0]=(send_data2>>24);
 comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_POS << 8), 
buffer, send_index);//
}

/*若要设置电调电流的上限和下限请使用上位机进行设置    */



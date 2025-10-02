#include "briter_encoder.h"

uint8_t Briter_TxData[8];

/**************************************************************
	@brief:		布瑞特绝对值编码器数据发送函数
	@param:		
	@retval: 		
	@supplement:	布瑞特绝对值编码器控制
**************************************************************/
uint8_t Encoder_Set_Function(Encoder_Struct *Encoder_Structure)            //布瑞特绝对值编码器控制
{
	uint32_t msgbox;
	CAN_TxHeaderTypeDef	Briter_Txmessage;

	Briter_Txmessage.StdId = Encoder_Structure->Encoder_ID;							             //编码器ID
	Briter_Txmessage.IDE		= Encoder_Structure->Data_Format;						             //标准帧格式 CAN_ID_STD 0
	Briter_Txmessage.RTR		= Encoder_Structure->Data_Type;							             //数据帧 CAN_RTR_DATA 0
	Briter_Txmessage.ExtId = 0;																		                   //扩展ID
	Briter_Txmessage.DLC 	= 8;				                       //传送的数据长度,字节数

	Briter_TxData[0]	= (Encoder_Structure->Front_Data[0]);
	Briter_TxData[1]	= (Encoder_Structure->Front_Data[1]);
	Briter_TxData[2]	= (Encoder_Structure->Front_Data[2]);	
	Briter_TxData[3]	= (Encoder_Structure->Front_Data[3]);
	Briter_TxData[4]	= (Encoder_Structure->Front_Data[4]);
	Briter_TxData[5]	= (Encoder_Structure->Front_Data[5]);
	Briter_TxData[6]	= (Encoder_Structure->Front_Data[6]);
	Briter_TxData[7]	= (Encoder_Structure->Front_Data[7]);
/******申请空闲消息盒子并提出发送申请******/
	HAL_CAN_AddTxMessage(&hcan2, &Briter_Txmessage, Briter_TxData, &msgbox);
/***************查询消息发送情况**************/
	while(HAL_CAN_AddTxMessage(&hcan2, &Briter_Txmessage, Briter_TxData, &msgbox)!=HAL_OK);

/******消息发送成功*****/
	return 0;
}

/**************************************************************
	@brief:		编码器当前角度查询函数
	@param:		
	@retval: 		
	@supplement:	布瑞特绝对值编码器控制
**************************************************************/
void encoder_read_angle(Encoder_Struct *control_Encoder, int8_t id)
{
	control_Encoder->Encoder_ID = id;
	control_Encoder->Data_Format = 0;
	control_Encoder->Data_Type = 0;
	control_Encoder->Front_Data[0] = 0x04;
	control_Encoder->Front_Data[1] = id;
	control_Encoder->Front_Data[2] = 0x01;
	control_Encoder->Front_Data[3] = 0x00;
	control_Encoder->Front_Data[4] = 0;
	control_Encoder->Front_Data[5] = 0;
	control_Encoder->Front_Data[6] = 0;
	control_Encoder->Front_Data[7] = 0;
	
  Encoder_Set_Function(control_Encoder);
}

/**************************************************************
	@brief:		编码器ID设置函数
	@param:		
	@retval: 		
	@supplement:	布瑞特绝对值编码器控制
**************************************************************/
void encoder_set_id(Encoder_Struct *control_Encoder, int8_t id)
{
	control_Encoder->Encoder_ID = id;
	control_Encoder->Data_Format = 0;
	control_Encoder->Data_Type = 0;
	control_Encoder->Front_Data[0] = 0x04;
	control_Encoder->Front_Data[1] = 0x01;//编码器初始ID为0x01,此数值随编码器ID变化而变化
	control_Encoder->Front_Data[2] = 0x02;
	control_Encoder->Front_Data[3] = id;//要设置的id
	control_Encoder->Front_Data[4] = 0;
	control_Encoder->Front_Data[5] = 0;
	control_Encoder->Front_Data[6] = 0;
	control_Encoder->Front_Data[7] = 0;
	
	Encoder_Set_Function(control_Encoder);
}

/**************************************************************
	@brief:		编码器波特率设置函数
	@param:		
	@retval: 		
	@supplement:	布瑞特绝对值编码器控制
**************************************************************/
void encoder_set_baud(Encoder_Struct *control_Encoder,int8_t id, int16_t baud)
{
	control_Encoder->Encoder_ID = id;
	control_Encoder->Data_Format = 0;
	control_Encoder->Data_Type = 0;
	control_Encoder->Front_Data[0] = 0x04;
	control_Encoder->Front_Data[1] = id;
	control_Encoder->Front_Data[2] = 0x03;
	if(baud==500)control_Encoder->Front_Data[3] = 0x00;
	if(baud==1000)control_Encoder->Front_Data[3] = 0x01;
	if(baud==250)control_Encoder->Front_Data[3] = 0x02;
	control_Encoder->Front_Data[4] = 0;
	control_Encoder->Front_Data[5] = 0;
	control_Encoder->Front_Data[6] = 0;
	control_Encoder->Front_Data[7] = 0;
	
	Encoder_Set_Function(control_Encoder);
}

/**************************************************************
	@brief:		编码器工作模式设置函数
	@param:		
	@retval: 		
	@supplement:	布瑞特绝对值编码器控制
**************************************************************/
void encoder_set_mode(Encoder_Struct *control_Encoder,int8_t id, int8_t mode)
{	
	control_Encoder->Encoder_ID = id;
	control_Encoder->Data_Format = 0;
	control_Encoder->Data_Type = 0;
	control_Encoder->Front_Data[0] = 0x04;
	control_Encoder->Front_Data[1] = id;
	control_Encoder->Front_Data[2] = 0x04;
	if(mode==reference_mode)control_Encoder->Front_Data[3] = 0x00;//查询模式
	if(mode==backhaul_mode)control_Encoder->Front_Data[3] = 0xAA;//自动回发模式
	control_Encoder->Front_Data[4] = 0;
	control_Encoder->Front_Data[5] = 0;
	control_Encoder->Front_Data[6] = 0;
	control_Encoder->Front_Data[7] = 0;
	
	Encoder_Set_Function(control_Encoder);
}

/**************************************************************
	@brief:		回传速度设置函数
	@param:		速度太快将无法设置其它参数，03E8为1000微秒
	@retval: 		
	@supplement:	布瑞特绝对值编码器控制
**************************************************************/
void encoder_set_speed(Encoder_Struct *control_Encoder,int8_t id, uint16_t speed)
{
	control_Encoder->Encoder_ID = id;
	control_Encoder->Data_Format = 0;
	control_Encoder->Data_Type = 0;
	control_Encoder->Front_Data[0] = 0x05;
	control_Encoder->Front_Data[1] = id;
	control_Encoder->Front_Data[2] = 0x05;
	control_Encoder->Front_Data[3] = speed && 0xff;//提取低8位
	control_Encoder->Front_Data[4] = speed >> 8;//获取高8位
	control_Encoder->Front_Data[5] = 0;
	control_Encoder->Front_Data[6] = 0;
	control_Encoder->Front_Data[7] = 0;
	
	Encoder_Set_Function(control_Encoder);
}

/**************************************************************
	@brief:		零点设置函数
	@param:		
	@retval: 		
	@supplement:	设定当前位置值为零点
**************************************************************/
void encoder_set_zero(Encoder_Struct *control_Encoder, int8_t id)//设定当前位置值为零点
{
	control_Encoder->Encoder_ID = id;
	control_Encoder->Data_Format = 0;
	control_Encoder->Data_Type = 0;
	control_Encoder->Front_Data[0] = 0x04;
	control_Encoder->Front_Data[1] = id;
	control_Encoder->Front_Data[2] = 0x06;
	control_Encoder->Front_Data[3] = 0x00;
	control_Encoder->Front_Data[4] = 0;
	control_Encoder->Front_Data[5] = 0;
	control_Encoder->Front_Data[6] = 0;
	control_Encoder->Front_Data[7] = 0;
	
	Encoder_Set_Function(control_Encoder);
}

/**************************************************************
	@brief:		计数方向设置函数
	@param:		
	@retval: 		
	@supplement:	设定计数递增方向
**************************************************************/
void encoder_set_direction(Encoder_Struct *control_Encoder,int8_t id, uint16_t direction)
{
	control_Encoder->Encoder_ID = id;
	control_Encoder->Data_Format = 0;
	control_Encoder->Data_Type = 0;
	control_Encoder->Front_Data[0] = 0x04;
	control_Encoder->Front_Data[1] = id;
	control_Encoder->Front_Data[2] = 0x07;
	if(direction==clockwise)
		control_Encoder->Front_Data[3] = 0x00;//编码器顺时针递增
	if(direction==anticlockwise)
		control_Encoder->Front_Data[3] = 0x01;//编码器逆时针递增
	control_Encoder->Front_Data[4] = 0;
	control_Encoder->Front_Data[5] = 0;
	control_Encoder->Front_Data[6] = 0;
	control_Encoder->Front_Data[7] = 0;
	
	Encoder_Set_Function(control_Encoder);
}

/**************************************************************
	@brief:		中点设置函数
	@param:		
	@retval: 		
	@supplement:	设定当前位置为中点
**************************************************************/
void encoder_set_half(Encoder_Struct *control_Encoder,int8_t id)
{
	control_Encoder->Encoder_ID = id;
	control_Encoder->Data_Format = 0;
	control_Encoder->Data_Type = 0;
	control_Encoder->Front_Data[0] = 0x04;
	control_Encoder->Front_Data[1] = id;
	control_Encoder->Front_Data[2] = 0x0C;
	control_Encoder->Front_Data[3] = 0x01;
	control_Encoder->Front_Data[4] = 0;
	control_Encoder->Front_Data[5] = 0;
	control_Encoder->Front_Data[6] = 0;
	control_Encoder->Front_Data[7] = 0;
	
	Encoder_Set_Function(control_Encoder);
}




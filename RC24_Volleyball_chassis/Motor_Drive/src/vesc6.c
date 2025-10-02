#include "vesc6.h"
#include "string.h"

/*���ڴ��vesc�ķ���ֵ*/
can_status_msg can_vescmsg[vesc6_number]={0};
can_status_msg2 can_vescmsg2[vesc6_number]={0};
can_status_msg3 can_vescmsg3[vesc6_number]={0};
can_status_msg4 can_vescmsg4[vesc6_number]={0};
can_status_msg5 can_vescmsg5[vesc6_number]={0};

/*ʵʱ�Ŀ��Ʊ���*/
volatile float vesc_current = 0.0f;
volatile float vesc_rpm = 0.0f;
volatile float vesc_pos = 0.0f;
volatile uint8_t vesc_mode=0;  //���Ĭ��Ϊ�ٶ�ģʽ


void comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len) 
{
  CAN_TxHeaderTypeDef TxMessage;
	uint32_t box;
	uint8_t Data[8];
	
  if (len > 8) 
	 {
    len = 8;                          //��෢��8���ֽڵ�����
   } 
   
 TxMessage.ExtId=id;                  // ������չID��29λ��
 TxMessage.IDE=CAN_ID_EXT;            // ��չ֡��ʽ
 TxMessage.RTR=0;                     // ����Ϊ����֡
 TxMessage.DLC=len;                   // ���͵����ݳ���,�ֽ���
 memcpy(Data, data, len);   // ��data��ַ�����ݣ�������TxMessage.Data�У��ֽ���Ϊlen

/******���������Ϣ���Ӳ������������******/
	HAL_CAN_AddTxMessage(&hcan2,&TxMessage,Data,&box);
/***************��ѯ��Ϣ�������**************/
	while(HAL_CAN_AddTxMessage(&hcan2,&TxMessage,Data,&box) != HAL_OK);   //����Ϊ������ԣ�д�ɷ���ʧ�ܳ���Ϳ�������ʽ
	
/******��Ϣ���ͳɹ�*****/
}

/**************************************************************
	@brief:		����������ռ�ձ�
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
	@brief:		����������ת��
	@param:		��λ��RPM
	@retval: 		
	@supplement:	
**************************************************************/
void vesc_can_set_rpm(uint8_t controller_id, float rpm)      
{
 int32_t send_index = 4;
 uint8_t buffer[4]={0};
 int32_t send_data2=0;
 send_data2=rpm*poles;    //��RPMת����ERPM
 buffer[3]=(send_data2)&0xff;
 buffer[2]=(send_data2>>8)&0xff;
 buffer[1]=(send_data2>>16)&0xff;
 buffer[0]=(send_data2>>24);
 comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_RPM << 8), 
buffer, send_index);//
}

/**************************************************************
	@brief:		��������������
	@param:		��λ��A
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
	@brief:		����������ɲ������
	@param:		��λ��A
	@retval: 		
	@supplement:	��������ɲ����������������
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
	@brief:		��������������λ��
	@param:		��λ����
	@retval: 		
	@supplement:	�ϵ�λ��Ϊ0�㣬�˶�����Ϊ�Ż����򣨼�����ǰΪ0�㣬Ŀ��Ϊ350��ʱ�������ѡ������ת10�㣩
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

/*��Ҫ���õ�����������޺�������ʹ����λ����������    */



#include "bsp_can.h"
#include "main.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

	/*CAN������������������*/
void can_filter_init(void)
{
	/*����һ����Ϊcan_filter_st��CAN_FilterTypeDef���͵ı���*/
  CAN_FilterTypeDef can_filter_st;
	
	/*�������������*/
  can_filter_st.FilterActivation = ENABLE;
	
	/*���ù������Ĺ���ģʽΪID������ģʽ*/
  can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
	
	/*���ù������ߴ�Ϊ32λ��ID�����붼Ϊ32λ*/
  can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
	
	/*��������ID��16λ�͵�16λ������Ϊ0����ʾƥ������ID�ĸ�16λ�͵�16λ*/
  can_filter_st.FilterIdHigh = 0x0000;
  can_filter_st.FilterIdLow = 0x0000;
	
	/*���ù������������16λ�͵�16λ��Ϊ0����ʾ�������κ�IDλ*/
  can_filter_st.FilterMaskIdHigh = 0x0000;
  can_filter_st.FilterMaskIdLow = 0x0000;
	
	/*ѡ��ʹ��CANӲ���ĵ�0��������*/
  can_filter_st.FilterBank = 0;
	
	/*����������������CAN����FIFO*/
  can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
	
	/*ʹ��HAL�⺯������CAN1�Ĺ�����*/
  HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
	
	/*����CAN1������*/
  HAL_CAN_Start(&hcan1);
	
	/*����CAN1�Ľ���FIFO��Ϣ�����ж�֪ͨ����FIFO0���ܵ��µ���Ϣʱ��������һ���ж�*/
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

	
	/*����ΪCAN2ͨ�Ź�������ʼ������*/
	
	/*��CAN������֧�ֶ����������ʱ�������Աָ���˴��ĸ���������ʼ����*/
  can_filter_st.SlaveStartFilterBank = 14;
	/*�����Աָ���˵�ǰ�������õĹ�������ı��*/
  can_filter_st.FilterBank = 14;
	/*����HAL�⺯������hcan2������*/
  HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
	/*����HAL�⺯��������hcan2 CAN������*/
  HAL_CAN_Start(&hcan2);
	/*����hcan2�Ľ���FIFO0��Ϣ�����ն�֪ͨ*/
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

}


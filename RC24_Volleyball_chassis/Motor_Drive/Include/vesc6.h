#ifndef __VESC6_H
#define __VESC6_H	 
#include "can.h"	

typedef enum {
CAN_PACKET_SET_DUTY = 0,
CAN_PACKET_SET_CURRENT,
CAN_PACKET_SET_CURRENT_BRAKE,
CAN_PACKET_SET_RPM,
CAN_PACKET_SET_POS,
CAN_PACKET_FILL_RX_BUFFER,
CAN_PACKET_FILL_RX_BUFFER_LONG,
CAN_PACKET_PROCESS_RX_BUFFER,
CAN_PACKET_PROCESS_SHORT_BUFFER,
CAN_PACKET_STATUS,
CAN_PACKET_SET_CURRENT_REL,
CAN_PACKET_SET_CURRENT_BRAKE_REL,
CAN_PACKET_SET_CURRENT_HANDBRAKE,
CAN_PACKET_SET_CURRENT_HANDBRAKE_REL,
CAN_PACKET_STATUS_2,
CAN_PACKET_STATUS_3,
CAN_PACKET_STATUS_4,
CAN_PACKET_PING,
CAN_PACKET_PONG,
CAN_PACKET_DETECT_APPLY_ALL_FOC,
CAN_PACKET_DETECT_APPLY_ALL_FOC_RES,
CAN_PACKET_CONF_CURRENT_LIMITS,
CAN_PACKET_CONF_STORE_CURRENT_LIMITS,
CAN_PACKET_CONF_CURRENT_LIMITS_IN,
CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN,
CAN_PACKET_CONF_FOC_ERPMS,
CAN_PACKET_CONF_STORE_FOC_ERPMS,
CAN_PACKET_STATUS_5
} CAN_PACKET_ID;   //vesc6各功能的对应扩展ID

#define poles 7.0f    //设置航模电机的磁极对数
#define vesc6_number 3  //挂在总线上的VESC6数量
#define vesc6_return_mode 1  //设置VESC6的回传数据模式（上位机也要对应做修改）
/*****射球速度控制*******/
#define shoot_speed1   4000  //4000
#define shoot_speed2   1500  //1500

/***********************/
typedef struct
{
int id;
uint32_t rx_time;
float rpm;
float current;
float duty;
}can_status_msg;

typedef struct
{
int id;
uint32_t rx_time;
float amp_hours;
float amp_hours_charged;
}can_status_msg2;


typedef struct
{
int id;
uint32_t rx_time;
float watt_hours;
float watt_hours_charged;
}can_status_msg3;


typedef struct
{
int id;
uint32_t rx_time;
float temp_fet;
float temp_motor;
float current_in;
float pid_pos_now;
}can_status_msg4;


typedef struct
{
int id;
uint32_t rx_time;
float v_in;
int32_t tacho_value;
}can_status_msg5;

//can_status_msg  *stat_tmp; 
//can_status_msg2 *stat_tmp_2;
//can_status_msg3 *stat_tmp_3;
//can_status_msg4 *stat_tmp_4;
//can_status_msg5 *stat_tmp_5;


void comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len); 
void vesc_can_set_duty(uint8_t controller_id, float duty);
void vesc_can_set_rpm(uint8_t controller_id, float rpm) ;
void vesc_can_set_current(uint8_t controller_id, float current);
void vesc_can_set_current_brake(uint8_t controller_id, float current);
void vesc_can_set_position(uint8_t controller_id, float position);
extern can_status_msg can_vescmsg[vesc6_number];
extern can_status_msg2 can_vescmsg2[vesc6_number];
extern can_status_msg3 can_vescmsg3[vesc6_number];
extern can_status_msg4 can_vescmsg4[vesc6_number];
extern can_status_msg5 can_vescmsg5[vesc6_number];

#endif


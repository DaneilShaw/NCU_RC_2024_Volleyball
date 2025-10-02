#ifndef __HTM_5046_H
#define __HTM_5046_H
#include "My_Includes.h"

/* NAN 表示不限制 */
#define INI8_NAN 0x80
#define INT16_NAN 0x8000
#define INT32_NAN 0x80000000
typedef struct
{
  uint32_t id;
  int16_t position;
  int16_t velocity;
  int16_t torque;
	
	float pos_rad;
	float angular_vel;
	
	float val;
	float tqe;
	float pos;
}motor_state_s;

#pragma anon_unions

//typedef struct
//{
//  union
//  {
//    motor_state_s motor;
//    uint8_t data[24];
//  };
//}motor_state_t;

//extern motor_state_t motor_state;
extern uint8_t motor_read_flag;

void CAN_Send_Msg(uint32_t id, uint8_t *msg, uint8_t len);

extern void HTMotor_Write(motor_state_s * ptr_state, float val, float tqe, float pos);
extern void motor_control_Pos(uint8_t id, int32_t pos, int16_t tqe);
//extern void motor_control_Vel(uint8_t id, int16_t vel, int16_t tqe);
extern void motor_control_Vel(uint8_t id, motor_state_s * ptr_state);
extern void motor_control_tqe(uint8_t id, int32_t tqe);
extern void motor_control_pos_val_tqe(uint8_t id, motor_state_s * ptr_state);
extern void motor_read(uint8_t id);


#endif


#ifndef Encoder_H
#define Encoder_H

#include "briter_encoder.h"
#include "struct_typedef.h"
#include "can.h"

#define reference_mode 0//查询模式
#define backhaul_mode 1//自动回发模式
#define clockwise 0
#define anticlockwise 1

typedef struct
{
	unsigned Data_Format:2;
	unsigned Data_Type:2;
	uint8_t Encoder_ID;
	int8_t Front_Data[8];
}Encoder_Struct;

uint8_t Encoder_Set_Function(Encoder_Struct *Encoder_Structure);
void encoder_set_zero(Encoder_Struct *control_Encoder, int8_t id);
void encoder_set_id(Encoder_Struct *control_Encoder, int8_t id);  
void encoder_read_angle(Encoder_Struct *control_Encoder, int8_t id);
void encoder_set_mode(Encoder_Struct *control_Encoder,int8_t id, int8_t mode) ; 
void encoder_set_baud(Encoder_Struct *control_Encoder,int8_t id, int16_t baud)	;
void encoder_set_speed(Encoder_Struct *control_Encoder,int8_t id, uint16_t speed);
void encoder_set_direction(Encoder_Struct *control_Encoder,int8_t id, uint16_t direction);
void encoder_set_half(Encoder_Struct *control_Encoder,int8_t id);

#endif


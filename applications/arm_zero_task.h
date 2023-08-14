#ifndef ARM_ZERO_H
#define ARM_ZERO_H

#include "struct_typedef.h"
#include "bsp_rs485.h"
#include "pid.h"
#include "TD.h"

typedef struct
{
	MOTOR_send   motor_send;
	MOTOR_recv   motor_recv;
	fp32 zero_angle;
	fp32 angle;
	fp32 angle_set;
	fp32 ramp;
	uint8_t motor_id;
}motor_move_t;


typedef struct
{
	motor_move_t arm_motor;
	TD_t arm_motro_TD;
}arm_move_t;

extern void arm_zero_task(void const * argument);


extern arm_move_t arm_move;
extern fp32 motor_angle_set;
extern fp32 motor_angle_init;

#endif



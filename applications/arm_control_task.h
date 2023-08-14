#ifndef ARM_CONTROL_TASK_H
#define ARM_CONTROL_TASK_H

#include "struct_typedef.h"

typedef enum
{
	INIT_POSITION = 0,
	LIFT_UP_POSITION_1 = 1,
	GO_TO_CATCH_POSITION = 2,
	CATCH_TARGET = 3,
	LIFT_UP_POSITION_2 = 4,
	PREPARE_STORE_POSITION = 5,
	STORE_POSITION = 6,
	RETURN_POSITION = 7,
}CATCH_SILVER_ORE_POSITION_e;

typedef enum
{
	INIT_POSITION_1 = 0,
	LIFT_UP_POSITION_1_1 = 1,
	GO_TO_CATCH_POSITION_1 = 2,
	CATCH_TARGET_1 = 3,
	LIFT_UP_POSITION_2_1 = 4,
	PREPARE_STORE_POSITION_1 = 5,
	STORE_POSITION_1 = 6,
	PREPARE_RETURN_POSITION_1_1 = 7,
	PREPARE_RETURN_POSITION_2_1 = 8,
	RETURN_POSITION_1 = 9,
}CATCH_SILVER_ORE_POSITION_1_e;

typedef enum
{
	INIT_POSITION_ROTATE_1 = 0,
	ROTATE_LIFT_1_POSITION_1 = 1,
	ROTATE_LIFT_2_POSITION_1 = 2,
	ROTATE_CHATH_TARGET_1 = 3,
	ROTATE_LIFT_3_POSITION_1 = 4,
	ROTETA_STORE_POSITION_1 = 5,
	ROTETA_RETURN_POSITION_1 = 6,
	ROTETA_RETURN_POSITION_2,
}rotate_ore_1_e;

typedef enum
{
	INIT_POSITION_EXCHANGE = 0,
	EXCHANGE_LIFT_1_POSITION = 1,
	EXCHANGE_LIFT_2_POSITION = 2,
	EXCHANGE_CHATH_TARGET = 3,
	EXCHANGE_LIFT_3_POSITION = 4,
	EXCHANGE_LIFT_4_POSITION = 5,
	EXCHANGE_PREPARE_POSITION = 6,
	EXCHANGE_POSITION = 7,
	PREPARE_RETURN_POSITION = 8,
	EXCHANGE_RETURN_POSITION = 9,
}exchange_ore_1_e;

typedef enum
{
	INIT_POSITION_GET = 0,
	PREPARE_GET_POSITION_1 = 1,
	PREPARE_GET_POSITION_2 = 2,
	GET_ORE_POSITION = 3,
	GET_LIFT_UP = 4,
}get_ore_position_e;

typedef struct
{
	fp32 motor_2_position;
	fp32 motor_3_position;
	fp32 motor_4_position;
	fp32 motor_5_position;
	fp32 motor_6_position;
}arm_control_t;


extern arm_control_t arm_control;
extern fp32 arm_2_set;
extern fp32 arm_3_set;
extern fp32 arm_2_compansate;
extern fp32 arm_3_compansate;
extern fp32 arm_4_compansate;
extern fp32 arm_5_compansate;
extern fp32 arm_6_compansate;

extern void arm_control_task(void const * argument);

#endif



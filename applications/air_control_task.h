#ifndef AIR_CONTROL_TASK_H
#define AIR_CONTROL_TASK_H

#include "struct_typedef.h"
#include "keyboard.h"

typedef enum
{
	CHASSIS_BE_UNCLOCK,
	CHASSIS_BE_CLOCK,
}chassis_clock_e; //µ×ÅÌËø¶¨Æø¸××´Ì¬

typedef enum
{
	PICTURE_POSITION_DOWN,
	PICTURE_POSITION_UP,
}picture_position_e;  //Í¼´«Î»ÖÃ×´Ì¬

typedef enum
{
	CLOSE,
	OPEN,
}sucker_e;  //Õæ¿Õ±Ã¹¤×÷×´Ì¬

typedef enum
{
	POWER_CLOSE,
	POWER_OPEN,
}send_power_e; //»úÐµ±ÛËÍµç×´Ì¬

typedef enum
{
	SERVO_POSITION_INIT,
	SERVO_POSITION_DISPLAY,
	SERVO_POSITION_RETURN,
}servo_position_e;

extern chassis_clock_e chassis_clock_flag;
extern picture_position_e picture_position_flag;
extern sucker_e sucker_flag;
extern mode_e mode_control_flag;

extern void air_control_task(void const * argument);

#endif


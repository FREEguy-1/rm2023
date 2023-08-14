#include "arm_zero_task.h"
#include "usart.h"
#include "bsp_rs485.h"
#include "cmsis_os.h"
#include "A_communicate_task.h"
#include "TD.H"
#include "air_control_task.h"
#include "chassis_task.h"
#include "pc_info_task.h"

static void arm_two_three_init(arm_move_t *arm_move_init);
static void arm_two_three_feedback_update(arm_move_t *arm_move_feedback_update);
static void arm_two_three_set_control(arm_move_t *arm_move_set_control);
//static void arm_two_three_control_loop(arm_move_t *arm_move_control_loop);

arm_move_t arm_move;
fp32 motor_angle_init;
fp32 motor_angle_set;

void arm_zero_task(void const * argument)
{
	arm_two_three_init(&arm_move);
	while(1)
	{
		arm_two_three_feedback_update(&arm_move);
		arm_two_three_set_control(&arm_move);
//		arm_two_three_control_loop(&arm_move);
		
//		modify_and_send_data_0(&arm_move.arm_motor[0].motor_send);
		modify_and_send_data_0(&arm_move.arm_motor.motor_send);
		osDelay(1);
	}
}

void arm_two_three_init(arm_move_t *arm_move_init)
{
	arm_move_init->arm_motor.motor_send.id = 0;
	arm_move_init->arm_motor.motor_send.Pos = 0.0f;
	arm_move_init->arm_motor.motor_send.mode =  0;	
	arm_move_init->arm_motor.motor_send.T 	 =	0;				
	arm_move_init->arm_motor.motor_send.W 	 =	0;				
	arm_move_init->arm_motor.motor_send.K_P  =	0.2;					
	arm_move_init->arm_motor.motor_send.K_W  =	3;
	modify_and_send_data_0(&arm_move_init->arm_motor.motor_send);
	osDelay(1000);
	arm_move_init->arm_motor.angle = arm_move_init->arm_motor.motor_recv.Pos;
	arm_move_init->arm_motor.angle_set =0.0f;
	motor_angle_init = 0.0f;
	arm_move_init->arm_motor.motor_id = 0;
	arm_move_init->arm_motor.ramp = 0;
	motor_angle_set = 0.0f;
}

void arm_two_three_feedback_update(arm_move_t *arm_move_feedback_update)
{
	arm_move_feedback_update->arm_motor.angle  = arm_move_feedback_update->arm_motor.motor_recv.Pos;
}

void arm_two_three_set_control(arm_move_t *arm_move_set_control)
{
//	if (*rx_motor_position == NULL)
//	{
//		return;
//	}
//	
		if(switch_is_down(chassis_move.chassis_RC->rc.s[0]))
		{
			arm_move_set_control->arm_motor.motor_send.id = 0;
			arm_move_set_control->arm_motor.motor_send.Pos = 0.0f;
			arm_move_set_control->arm_motor.motor_send.mode =  0;	
			arm_move_set_control->arm_motor.motor_send.T 	 =	0;				
			arm_move_set_control->arm_motor.motor_send.W 	 =	0;				
			arm_move_set_control->arm_motor.motor_send.K_P  =	0.2;					
			arm_move_set_control->arm_motor.motor_send.K_W  =	3;
			TD_init(&arm_move_set_control->arm_motro_TD, 10.0f, 2.0f, 0.001f, arm_move_set_control->arm_motor.angle);
			motor_angle_init = arm_move_set_control->arm_motor.angle;
		}
		else if(mode_control_flag == FIRST_MODE || mode_control_flag == SECOND_MODE)
		{
			TD_calc(&arm_move_set_control->arm_motro_TD,motor_angle_init+arm_move_set_control->arm_motor.angle_set*1.5f);
			arm_move_set_control->arm_motor.motor_send.id = 0;
			arm_move_set_control->arm_motor.motor_send.Pos = (fp32)(arm_move_set_control->arm_motro_TD.x);
			arm_move_set_control->arm_motor.motor_send.mode =  10;	
			arm_move_set_control->arm_motor.motor_send.T 	 =	0;				
			arm_move_set_control->arm_motor.motor_send.W 	 =	0;				
			arm_move_set_control->arm_motor.motor_send.K_P  =	0.8;					
			arm_move_set_control->arm_motor.motor_send.K_W  =	8.5;
		}
		else if(mode_control_flag == THIRD_MODE)
		{
			if(chassis_move.chassis_RC->key.F == 1 && chassis_move.chassis_RC->rc.ch[1] > 0)
			{
				motor_angle_set += 0.0005f;
			}
			if(chassis_move.chassis_RC->key.F == 1 && chassis_move.chassis_RC->rc.ch[1] < 0)
			{
				motor_angle_set -= 0.0005f;
			}
			TD_calc(&arm_move_set_control->arm_motro_TD,motor_angle_init+pc_receive_msg.rx_data.motor1_position*1.5f);
//			TD_calc(&arm_move_set_control->arm_motro_TD,motor_angle_init+motor_angle_set*1.5f);
			arm_move_set_control->arm_motor.motor_send.id = 0;
			arm_move_set_control->arm_motor.motor_send.Pos = (fp32)(arm_move_set_control->arm_motro_TD.x);
			arm_move_set_control->arm_motor.motor_send.mode =  10;	
			arm_move_set_control->arm_motor.motor_send.T 	 =	0;				
			arm_move_set_control->arm_motor.motor_send.W 	 =	0;				
			arm_move_set_control->arm_motor.motor_send.K_P  =	0.8;					
			arm_move_set_control->arm_motor.motor_send.K_W  =	8.5;
		}
}




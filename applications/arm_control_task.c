#include "arm_control_task.h"
#include "arm_zero_task.h"
#include "A_communicate_task.h"
#include "air_control_task.h"
#include "keyboard.h"
#include "cmsis_os.h"
#include "pc_info_task.h"
#include "arm_math.h"
#include "math.h"
#include "chassis_task.h"


static void arm_control_init(all_key_t *arm_control_key_init);
static void arm_control_key_check(all_key_t *arm_cnotrol_key_check);
static void arm_move_right_posiiton(arm_move_t *arm_zero_move, arm_control_t *arm_control_position);
static void arm_control_flag_set(all_key_t *arm_cnotrol_flag_set);
static void arm_control_set(arm_move_t *arm_zero_control_set, arm_control_t *arm_control_set);
static void arm_resolve(arm_control_t *arm_control_position);

uint8_t arm_move_flag;
arm_control_t arm_control;
CATCH_SILVER_ORE_POSITION_e catch_silver_flag;
get_ore_position_e get_ore_flag;
CATCH_SILVER_ORE_POSITION_1_e catch_silver_1_flag;
rotate_ore_1_e rotete_1_flag;
uint16_t move_calculate_flag;
uint8_t observe_flag;
exchange_ore_1_e zero_exchange_flag;
fp32 position_x;
fp32 position_z;
fp32 arm_2_compansate;
fp32 arm_3_compansate;
fp32 arm_4_compansate;
fp32 arm_5_compansate;
fp32 arm_6_compansate;
fp32 arm_5_temp;
fp32 arm_2_set;
fp32 arm_3_set;

void arm_control_task(void const * argument)
{
	arm_control_init(&all_key);
	while(1)
	{
		arm_control_key_check(&all_key);
		arm_control_flag_set(&all_key);
		arm_move_right_posiiton(&arm_move, &arm_control);
		arm_resolve(&arm_control);
		arm_control_set(&arm_move, &arm_control);
		osDelay(2);
	}
}

void arm_control_init(all_key_t *arm_control_key_init)
{
	catch_silver_flag = INIT_POSITION;
	catch_silver_1_flag = INIT_POSITION_1;
	rotete_1_flag = INIT_POSITION_ROTATE_1;
	zero_exchange_flag = INIT_POSITION_EXCHANGE;
	get_ore_flag = INIT_POSITION_GET;
	move_calculate_flag = 0;
	arm_control.motor_2_position = 0.0f;
	arm_control.motor_3_position = 0.0f;
	arm_control.motor_4_position = 0.0f;
	arm_control.motor_5_position = 0.0f;
	arm_control.motor_6_position = 0.0f;
//	key_init(&arm_control_key_init->first_mode_key.catch_silver_ore_key, Z);
	key_init(&arm_control_key_init->first_mode_key.catch_silver_ore_key_1, X);
	key_init(&arm_control_key_init->first_mode_key.observe_move, Z);
	key_init(&arm_control_key_init->first_mode_key.rotate_ore_key_1, C);
	key_init(&arm_control_key_init->first_mode_key.zero_exchange, F);
	key_init(&arm_control_key_init->first_mode_key.clock_version_position, V);
	key_init(&arm_control_key_init->first_mode_key.get_ore_position,B);
	key_init(&arm_control_key_init->first_mode_key.version_move,Z);
	key_init(&arm_control_key_init->first_mode_key.version_move_1,X);
	key_init(&arm_control_key_init->first_mode_key.go_back,G);
	position_x = 0;
	position_z = 0;
	observe_flag = 0;
	arm_5_temp = 0.0f;
	arm_2_compansate = 0.0f;
	arm_3_compansate = 0.0f;
	arm_4_compansate = 0.0f;
	arm_5_compansate = 0.0f;
	arm_6_compansate = 0.0f;
	arm_2_set = 0.0f;
	arm_3_set = 0.0f;
}

void arm_resolve(arm_control_t *arm_control_position)
{
	if(mode_control_flag == FIRST_MODE&&chassis_move.chassis_RC->key.CTRL==0)
	{
		position_x = -200.0f*arm_cos_f32(rx_motor_position[0])+296.95f*arm_cos_f32(-rx_motor_position[1]+0.264f+rx_motor_position[0]);
		position_z = 200.0f*arm_sin_f32(-rx_motor_position[0])+296.95f*arm_sin_f32(-rx_motor_position[1]+0.264f+rx_motor_position[0]);
		arm_5_temp = 0.0f;
		arm_2_compansate = 0.0f;
		arm_3_compansate = 0.0f;
		arm_4_compansate = 0.0f;
		arm_5_compansate = 0.0f;
		arm_6_compansate = 0.0f;
	}
	else if(mode_control_flag == FIRST_MODE&&chassis_move.chassis_RC->key.CTRL == 1)
	{
		if(chassis_move.chassis_RC->key.W)
		{
			arm_5_temp += 0.002f;
		}
		else if(chassis_move.chassis_RC->key.S)
		{
			arm_5_temp -= 0.002f;
		}
		arm_5_compansate = arm_5_temp;
		position_x = -200.0f*arm_cos_f32(rx_motor_position[0])+296.95f*arm_cos_f32(-rx_motor_position[1]+0.264f+rx_motor_position[0]);
		position_z = 200.0f*arm_sin_f32(-rx_motor_position[0])+296.95f*arm_sin_f32(-rx_motor_position[1]+0.264f+rx_motor_position[0]);
		arm_2_compansate = 0.0f;
		arm_3_compansate = 0.0f;
		arm_4_compansate = 0.0f;
		arm_6_compansate = 0.0f;
	}
	else if(mode_control_flag == SECOND_MODE)
	{
		if(chassis_move.chassis_RC->key.W)
		{
			arm_5_temp += 0.002f;
		}
		else if(chassis_move.chassis_RC->key.S)
		{
			arm_5_temp -= 0.002f;
		}
		if(chassis_move.chassis_RC->key.A)
		{
			arm_4_compansate -= 0.002f;
		}
		else if(chassis_move.chassis_RC->key.D)
		{
			arm_4_compansate += 0.002f;
		}
		if(chassis_move.chassis_RC->key.Q)
		{
			arm_6_compansate += 0.002f;
		}
		else if(chassis_move.chassis_RC->key.E)
		{
			arm_6_compansate -= 0.002f;
		}
		if(chassis_move.chassis_RC->key.Z)
		{
			position_x += 1.0f;
		}
		else if(chassis_move.chassis_RC->key.X)
		{
			position_x -= 1.0f;
		}
		if(chassis_move.chassis_RC->key.C)
		{
			position_z += 1.0f;
		}
		else if(chassis_move.chassis_RC->key.V)
		{
			position_z -= 1.0f;
		}
		//(position_x/200.0f) -  3.14f-acosf
//		arm_3_set = ((position_x/200.0f)-((200.0f*200.0f + 296.95f*296.95f - (position_x*position_x + position_z*position_z))/(2*200.0f*296.95f)));
//		arm_2_set = ((((200.0f*200.0f+296.95f*296.95f-position_x*position_x-position_z*position_z))/(2*200.0f*200.0f)));
			arm_2_set = atan(position_z/position_x) + acos((200.0f*200.0f + position_z*position_z + position_x*position_x - 296.95f*296.95f)/(2*200.0f*sqrt(position_z*position_z+position_x*position_x)));
			arm_3_set = acos((296.95f*296.95f + position_z*position_z + position_x*position_x - 200.0f*200.0f)/(2*296.95f*sqrt(position_z*position_z+position_x*position_x)))-atan(position_z/position_x);
			arm_3_set = -arm_2_set-arm_3_set-0.264f;
			arm_2_set = -(3.1415926f-arm_2_set);
			arm_3_set = -(3.1415926f+arm_3_set);
			if(arm_3_set > 0 || arm_3_set<-3.14)
			{
				arm_3_set = -1.57;
				arm_2_set = -1.57;
				position_x = -200.0f*arm_cos_f32(arm_2_set)+296.95f*arm_cos_f32(-arm_3_set+0.264f+arm_2_set);
				position_z = 200.0f*arm_sin_f32(-arm_2_set)+296.95f*arm_sin_f32(-arm_3_set+0.264f+arm_2_set);
			}
			if(arm_2_set > 0 || arm_2_set<-3.14)
			{
				arm_3_set = -1.57;
				arm_2_set = -1.57;
				position_x = -200.0f*arm_cos_f32(arm_2_set)+296.95f*arm_cos_f32(-arm_3_set+0.264f+arm_2_set);
				position_z = 200.0f*arm_sin_f32(-arm_2_set)+296.95f*arm_sin_f32(-arm_3_set+0.264f+arm_2_set);
			}
//		arm_3_compansate = arm_3_set - rx_motor_position[1];
//		arm_2_compansate = arm_2_set - rx_motor_position[0];
		arm_5_compansate = arm_3_set - arm_2_set + arm_5_temp;
	}
	else if(mode_control_flag == THIRD_MODE)
	{
		if(chassis_move.chassis_RC->key.CTRL==0)
		{
			position_x = -200.0f*arm_cos_f32(rx_motor_position[0])+296.95f*arm_cos_f32(-rx_motor_position[1]+0.264f+rx_motor_position[0]);
			position_z = 200.0f*arm_sin_f32(-rx_motor_position[0])+296.95f*arm_sin_f32(-rx_motor_position[1]+0.264f+rx_motor_position[0]);
			arm_5_temp = 0.0f;
			arm_2_compansate = 0.0f;
			arm_3_compansate = 0.0f;
			arm_4_compansate = 0.0f;
			arm_5_compansate = 0.0f;
			arm_6_compansate = 0.0f;
		}
		else if(chassis_move.chassis_RC->key.CTRL == 1)
		{
			if(chassis_move.chassis_RC->key.W)
			{
				arm_5_temp += 0.002f;
			}
			else if(chassis_move.chassis_RC->key.S)
			{
				arm_5_temp -= 0.002f;
			}
			if(chassis_move.chassis_RC->key.A)
			{
				arm_4_compansate -= 0.002f;
			}
			else if(chassis_move.chassis_RC->key.D)
			{
				arm_4_compansate += 0.002f;
			}
			if(chassis_move.chassis_RC->key.Q)
			{
				arm_6_compansate += 0.002f;
			}
			else if(chassis_move.chassis_RC->key.E)
			{
				arm_6_compansate -= 0.002f;
			}
			if(chassis_move.chassis_RC->key.Z)
			{
				position_x += 1.0f;
			}
			else if(chassis_move.chassis_RC->key.X)
			{
				position_x -= 1.0f;
			}
			if(chassis_move.chassis_RC->key.C)
			{
				position_z += 1.0f;
			}
			else if(chassis_move.chassis_RC->key.V)
			{
				position_z -= 1.0f;
			}
			if(position_x < 50)
			{
				position_x = 50;
			}
			//(position_x/200.0f) -  3.14f-acosf
	//		arm_3_set = ((position_x/200.0f)-((200.0f*200.0f + 296.95f*296.95f - (position_x*position_x + position_z*position_z))/(2*200.0f*296.95f)));
	//		arm_2_set = ((((200.0f*200.0f+296.95f*296.95f-position_x*position_x-position_z*position_z))/(2*200.0f*200.0f)));
			arm_2_set = atan(position_z/position_x) + acos((200.0f*200.0f + position_z*position_z + position_x*position_x - 296.95f*296.95f)/(2*200.0f*sqrt(position_z*position_z+position_x*position_x)));
			arm_3_set = acos((296.95f*296.95f + position_z*position_z + position_x*position_x - 200.0f*200.0f)/(2*296.95f*sqrt(position_z*position_z+position_x*position_x)))-atan(position_z/position_x);
			arm_3_set = -arm_2_set-arm_3_set-0.264f;
			arm_2_set = -(3.1415926f-arm_2_set);
			arm_3_set = -(3.1415926f+arm_3_set);
			if(arm_3_set > 0 || arm_3_set<-3.14)
			{
				arm_3_set = -1.57;
				arm_2_set = -1.57;
				position_x = -200.0f*arm_cos_f32(arm_2_set)+296.95f*arm_cos_f32(-arm_3_set+0.264f+arm_2_set);
				position_z = 200.0f*arm_sin_f32(-arm_2_set)+296.95f*arm_sin_f32(-arm_3_set+0.264f+arm_2_set);
			}
			if(arm_2_set > 0 || arm_2_set<-3.14)
			{
				arm_3_set = -1.57;
				arm_2_set = -1.57;
				position_x = -200.0f*arm_cos_f32(arm_2_set)+296.95f*arm_cos_f32(-arm_3_set+0.264f+arm_2_set);
				position_z = 200.0f*arm_sin_f32(-arm_2_set)+296.95f*arm_sin_f32(-arm_3_set+0.264f+arm_2_set);
			}
			arm_5_compansate = arm_5_temp;
		}
	}
}

void arm_control_key_check(all_key_t *arm_cnotrol_key_check)
{
	if(mode_control_flag == FIRST_MODE)
	{
//		key_itself_press_num(&(arm_cnotrol_key_check->first_mode_key.catch_silver_ore_key), 2);
		key_itself_press_num(&(arm_cnotrol_key_check->first_mode_key.catch_silver_ore_key_1), 2);
		key_itself_press_num(&(arm_cnotrol_key_check->first_mode_key.rotate_ore_key_1), 2);
		key_itself_press_num(&(arm_cnotrol_key_check->first_mode_key.zero_exchange), 2);
		key_itself_press_num(&(arm_cnotrol_key_check->first_mode_key.observe_move),2);
		key_itself_press_num(&(arm_cnotrol_key_check->first_mode_key.clock_version_position),2);
		key_itself_press_num(&(arm_cnotrol_key_check->first_mode_key.get_ore_position),2);
		key_itself_press_num(&(arm_cnotrol_key_check->first_mode_key.go_back),2);
	}
	else if(mode_control_flag == THIRD_MODE)
	{
		get_ore_flag = INIT_POSITION_GET;
		key_itself_press_num(&(arm_cnotrol_key_check->first_mode_key.version_move),2);
		key_itself_press_num(&(arm_cnotrol_key_check->first_mode_key.version_move_1),2);
	}
}


void arm_control_flag_set(all_key_t *arm_cnotrol_flag_set)
{
//	if(arm_cnotrol_flag_set->first_mode_key.catch_silver_ore_key.itself.mode != arm_cnotrol_flag_set->first_mode_key.catch_silver_ore_key.itself.last_mode)
//	{
//		if(catch_silver_flag == 0)
//		{
//			catch_silver_flag = LIFT_UP_POSITION_1;
//		}
//		else
//		{
//			catch_silver_flag = INIT_POSITION;
//		}
//	}
	if(arm_cnotrol_flag_set->first_mode_key.catch_silver_ore_key_1.itself.mode != arm_cnotrol_flag_set->first_mode_key.catch_silver_ore_key_1.itself.last_mode)
	{
		if(catch_silver_1_flag == INIT_POSITION_1)
		{
			catch_silver_1_flag = LIFT_UP_POSITION_1_1;
		}
		else
		{
			catch_silver_1_flag = INIT_POSITION_1;
		}
	}
	if(arm_cnotrol_flag_set->first_mode_key.rotate_ore_key_1.itself.mode != arm_cnotrol_flag_set->first_mode_key.rotate_ore_key_1.itself.last_mode)
	{
		if(rotete_1_flag == INIT_POSITION_ROTATE_1)
		{
			rotete_1_flag = ROTATE_LIFT_1_POSITION_1;
		}
		else
		{
			rotete_1_flag = INIT_POSITION_ROTATE_1;
		}
	}
	if(arm_cnotrol_flag_set->first_mode_key.zero_exchange.itself.mode != arm_cnotrol_flag_set->first_mode_key.zero_exchange.itself.last_mode)
	{
		zero_exchange_flag = EXCHANGE_LIFT_1_POSITION;
	}
	if(arm_cnotrol_flag_set->first_mode_key.go_back.itself.mode != arm_cnotrol_flag_set->first_mode_key.go_back.itself.last_mode)
	{
		zero_exchange_flag = PREPARE_RETURN_POSITION;
	}
	if(arm_cnotrol_flag_set->first_mode_key.get_ore_position.itself.mode != arm_cnotrol_flag_set->first_mode_key.get_ore_position.itself.last_mode)
	{
		if(get_ore_flag == INIT_POSITION_GET)
		{
			get_ore_flag = PREPARE_GET_POSITION_1;
			observe_flag = 0;
		}
		else
		{
			get_ore_flag = INIT_POSITION_GET;
		}
	}

	if(arm_cnotrol_flag_set->first_mode_key.version_move.itself.mode != arm_cnotrol_flag_set->first_mode_key.version_move.itself.last_mode)
	{
		if(pc_send_msg.tx_data.pc_flag == 0||pc_send_msg.tx_data.pc_flag == 5)
		{
			pc_send_msg.tx_data.pc_flag = 1;
		}
		else if(pc_send_msg.tx_data.pc_flag == 1)
		{
			pc_send_msg.tx_data.pc_flag = 0;
		}
	}
	if(arm_cnotrol_flag_set->first_mode_key.version_move_1.itself.mode != arm_cnotrol_flag_set->first_mode_key.version_move_1.itself.last_mode)
	{
		if(pc_send_msg.tx_data.pc_flag == 0||pc_send_msg.tx_data.pc_flag == 5)
		{
			pc_send_msg.tx_data.pc_flag = 2;
		}
		else if(pc_send_msg.tx_data.pc_flag == 2)
		{
			pc_send_msg.tx_data.pc_flag = 0;
		}
	}
	if(arm_cnotrol_flag_set->first_mode_key.clock_version_position.itself.mode != arm_cnotrol_flag_set->first_mode_key.clock_version_position.itself.last_mode)
	{
		if(pc_send_msg.tx_data.pc_flag == 4)
		{
			pc_send_msg.tx_data.pc_flag = 5;
		}
	}
	
	if(arm_cnotrol_flag_set->first_mode_key.observe_move.itself.mode != arm_cnotrol_flag_set->first_mode_key.observe_move.itself.last_mode)
	{
		if(observe_flag == 0)
		{
			pc_send_msg.tx_data.pc_flag = 4;
			observe_flag = 1;
		}
		else
		{
			observe_flag = 4;
		}
	}
	
//	if(arm_cnotrol_flag_set->first_mode_key.catch_silver_ore_key.itself.mode != arm_cnotrol_flag_set->first_mode_key.catch_silver_ore_key.itself.last_mode)
//	{
//		if(pc_send_msg.tx_data.pc_flag == 0)
//		{
//			pc_send_msg.tx_data.pc_flag = 2;
//		}
//	}
}

 void arm_move_right_posiiton(arm_move_t *arm_zero_move, arm_control_t *arm_control_position)
 {
	 if(arm_zero_move->arm_motor.angle_set*1.5f+ motor_angle_init - arm_zero_move->arm_motor.angle < 0.3f
		 &&arm_zero_move->arm_motor.angle_set*1.5f+motor_angle_init - arm_zero_move->arm_motor.angle > -0.3f
		 &&arm_control_position->motor_2_position - rx_motor_position[0] < 0.3f
		 &&arm_control_position->motor_2_position - rx_motor_position[0] > -0.3f
		 &&arm_control_position->motor_3_position - rx_motor_position[1] < 0.3f
		 &&arm_control_position->motor_3_position - rx_motor_position[1] > -0.3f
		 &&arm_control_position->motor_4_position - rx_motor_position[2] < 0.3f
		 &&arm_control_position->motor_4_position - rx_motor_position[2] > -0.3f
		 &&arm_control_position->motor_5_position - rx_motor_position[3] < 0.3f
		 &&arm_control_position->motor_5_position - rx_motor_position[3] > -0.3f
		 &&arm_control_position->motor_6_position - rx_motor_position[4] < 0.3f
		 &&arm_control_position->motor_6_position - rx_motor_position[4] > -0.3f)
	 {
			arm_move_flag = 1;
	 }
	 else
	 {
			arm_move_flag = 0;
	 }
	 
	 if (arm_move_flag ==1)
	 {
			move_calculate_flag++;
	 }
	 else
	 {
			move_calculate_flag = 0;
	 }
 }

 void arm_control_set(arm_move_t *arm_zero_control_set, arm_control_t *arm_control_set)
 {
		if(catch_silver_1_flag == LIFT_UP_POSITION_1_1)
		{
			arm_zero_control_set->arm_motor.angle_set = 0.000f;
			arm_control_set->motor_2_position = -0.586f;
			arm_control_set->motor_3_position =  -0.890f;
			arm_control_set->motor_4_position = -0.047f;
			arm_control_set->motor_5_position = -0.3149f;
			arm_control_set->motor_6_position = 0.5954f;
			if(move_calculate_flag == 10)
			{
				catch_silver_1_flag = GO_TO_CATCH_POSITION_1;
				arm_move_flag = 0;
				move_calculate_flag = 0;
			}
		}
		else if(catch_silver_1_flag==GO_TO_CATCH_POSITION)
		{
			arm_zero_control_set->arm_motor.angle_set = 1.575f;
			arm_control_set->motor_2_position = -0.586f;
			arm_control_set->motor_3_position = -0.890;
			arm_control_set->motor_4_position = 0.022f;
			arm_control_set->motor_5_position = -0.231f;
			arm_control_set->motor_6_position = 0.375f;
			if(move_calculate_flag == 10)
			{
				picture_position_flag = PICTURE_POSITION_UP;
				catch_silver_1_flag = CATCH_TARGET_1;
				arm_move_flag = 0;
				move_calculate_flag = 0;
			}
		}
		else if(catch_silver_1_flag==CATCH_TARGET)
		{
			arm_zero_control_set->arm_motor.angle_set = 1.575f;
			arm_control_set->motor_2_position = -0.850f;
			arm_control_set->motor_3_position = -0.800f;
			arm_control_set->motor_4_position = 0.000f;
			arm_control_set->motor_5_position = 0.040f;
			arm_control_set->motor_6_position = 0.375f;
			if(move_calculate_flag == 10)
			{
				sucker_flag = OPEN;
			}
			if(move_calculate_flag >= 500&&mode_control_flag == FIRST_MODE)
			{
				catch_silver_1_flag = LIFT_UP_POSITION_2_1;
				arm_move_flag = 0;
				move_calculate_flag = 0;
			}
		}
		else if(catch_silver_1_flag==LIFT_UP_POSITION_2)
		{
			arm_zero_control_set->arm_motor.angle_set = 1.557f;
			arm_control_set->motor_2_position = -0.800f;
			arm_control_set->motor_3_position = -2.084f;
			arm_control_set->motor_4_position =	0.007f;
			arm_control_set->motor_5_position = -0.000f;
			arm_control_set->motor_6_position =	5.085f;
			if(move_calculate_flag == 10)
			{
				catch_silver_1_flag = PREPARE_STORE_POSITION_1;
				arm_move_flag = 0;
				move_calculate_flag = 0;
			}
		}
		else if(catch_silver_1_flag==PREPARE_STORE_POSITION)
		{
			arm_zero_control_set->arm_motor.angle_set = 3.1415f;
			arm_control_set->motor_2_position = -1.6f;
			arm_control_set->motor_3_position = -2.0455f;
			arm_control_set->motor_4_position = -1.0f;
			arm_control_set->motor_5_position = 1.0f;
			arm_control_set->motor_6_position =5.085f;
			if(move_calculate_flag == 10)
			{
				catch_silver_1_flag = STORE_POSITION_1;
				arm_move_flag = 0;
				move_calculate_flag = 0;
			}
		}
		else if(catch_silver_1_flag==STORE_POSITION)
		{
			arm_zero_control_set->arm_motor.angle_set = 3.2515f;
			arm_control_set->motor_2_position = -0.80f;
			arm_control_set->motor_3_position = -0.30f;
			arm_control_set->motor_4_position = -1.47f;
			arm_control_set->motor_5_position = 1.52f;
			arm_control_set->motor_6_position = 4.905f;
			if(move_calculate_flag == 100)
			{
				sucker_flag = CLOSE;
			}
			if(move_calculate_flag ==250)
			{
				catch_silver_1_flag = PREPARE_RETURN_POSITION_1_1;
				arm_move_flag = 0;
				move_calculate_flag = 0;
			}
		}
		else if(catch_silver_1_flag==PREPARE_RETURN_POSITION_1_1)
		{
			arm_zero_control_set->arm_motor.angle_set = 3.1415f;
			arm_control_set->motor_2_position = -1.6f;
			arm_control_set->motor_3_position = -3.0455f;
			arm_control_set->motor_4_position = 0.0f;
			arm_control_set->motor_5_position = 0.0f;
			arm_control_set->motor_6_position = 0.0f;
			if(move_calculate_flag == 10)
			{
				picture_position_flag = PICTURE_POSITION_DOWN;
				catch_silver_1_flag = PREPARE_RETURN_POSITION_2_1;
				arm_move_flag = 0;
				move_calculate_flag = 0;
			}
		}
		else if(catch_silver_1_flag==PREPARE_RETURN_POSITION_2_1)
		{
			arm_zero_control_set->arm_motor.angle_set = 0.0f;
			arm_control_set->motor_2_position = -1.6f;
			arm_control_set->motor_3_position = -3.0455f;
			arm_control_set->motor_4_position = 0.0f;
			arm_control_set->motor_5_position = 0.0f;
			arm_control_set->motor_6_position = 0.0f;
			if(move_calculate_flag == 10)
			{
				catch_silver_1_flag = RETURN_POSITION_1;
				arm_move_flag = 0;
				move_calculate_flag = 0;
			}
		}
		else if(catch_silver_1_flag==RETURN_POSITION_1)
		{
			arm_zero_control_set->arm_motor.angle_set = 0.0f;
			arm_control_set->motor_2_position = 0.0f;
			arm_control_set->motor_3_position = 0.0f;
			arm_control_set->motor_4_position = 0.0f;
			arm_control_set->motor_5_position = 0.0f;
			arm_control_set->motor_6_position = 0.0f;
			if(move_calculate_flag == 10)
			{
				catch_silver_1_flag = INIT_POSITION_1;
				arm_move_flag = 0;
				move_calculate_flag = 0;
			}
		}
		
/*×ª×ó¿ó*/
		if(rotete_1_flag == ROTATE_LIFT_1_POSITION_1)
		{
			arm_zero_control_set->arm_motor.angle_set = 0.000f;
			arm_control_set->motor_2_position = -1.57f;
			arm_control_set->motor_3_position =  -3.10f;
			arm_control_set->motor_4_position = -0.047f;
			arm_control_set->motor_5_position = -0.0f;
			arm_control_set->motor_6_position = 0.5954f;
			if(move_calculate_flag == 2)
			{
				rotete_1_flag = ROTATE_LIFT_2_POSITION_1;
				arm_move_flag = 0;
				move_calculate_flag = 0;
			}
		}
		else if(rotete_1_flag==ROTATE_LIFT_2_POSITION_1)
		{
			arm_zero_control_set->arm_motor.angle_set = 3.1415f;
			arm_control_set->motor_2_position = -1.57f;
			arm_control_set->motor_3_position = -3.10f;
			arm_control_set->motor_4_position =	0.00f;
			arm_control_set->motor_5_position = -0.000f;
			arm_control_set->motor_6_position = 3.14f;
			if(move_calculate_flag == 2)
			{
				rotete_1_flag = ROTATE_CHATH_TARGET_1;
				arm_move_flag = 0;
				move_calculate_flag = 0;
			}
		}
		else if(rotete_1_flag==ROTATE_CHATH_TARGET_1)
		{
			arm_zero_control_set->arm_motor.angle_set = 4.1615f;
			arm_control_set->motor_2_position = -1.333f;
			arm_control_set->motor_3_position = -0.993f;
			arm_control_set->motor_4_position = 0.0f;
			arm_control_set->motor_5_position = -1.395f;
			arm_control_set->motor_6_position = 3.14f;
			if(move_calculate_flag == 50)
			{
				sucker_flag = OPEN;
			}
			if(move_calculate_flag == 70 )
			{
				rotete_1_flag = ROTATE_LIFT_3_POSITION_1;
				arm_move_flag = 0;
				move_calculate_flag = 0;
			}
		}
		else if(rotete_1_flag==ROTATE_LIFT_3_POSITION_1)
		{
			arm_zero_control_set->arm_motor.angle_set =  3.1415f;
			arm_control_set->motor_2_position = -1.2f;
			arm_control_set->motor_3_position = -1.80f;
			arm_control_set->motor_4_position =	0.00f;
			arm_control_set->motor_5_position = -0.000f;
			arm_control_set->motor_6_position =	5.085f;
			if(move_calculate_flag == 2)
			{
				picture_position_flag = PICTURE_POSITION_UP;
				rotete_1_flag = ROTETA_STORE_POSITION_1;
				arm_move_flag = 0;
				move_calculate_flag = 0;
			}
		}
		else if(rotete_1_flag==ROTETA_STORE_POSITION_1)
		{
			arm_zero_control_set->arm_motor.angle_set = 3.2515f;
			arm_control_set->motor_2_position = -0.80f;
			arm_control_set->motor_3_position = -0.30f;
			arm_control_set->motor_4_position = -1.47f;
			arm_control_set->motor_5_position = 1.52f;
			arm_control_set->motor_6_position = 5.185f;
			if(move_calculate_flag == 50)
			{
				sucker_flag = CLOSE;
			}
			if(move_calculate_flag == 70)
			{
				rotete_1_flag = ROTETA_RETURN_POSITION_1;
				arm_move_flag = 0;
				move_calculate_flag = 0;
			}
		}
		else if(rotete_1_flag==ROTETA_RETURN_POSITION_1)
		{
			arm_zero_control_set->arm_motor.angle_set = 3.1415f;
			arm_control_set->motor_2_position = -1.57f;
			arm_control_set->motor_3_position = -3.0f;
			arm_control_set->motor_4_position = -0.0f;
			arm_control_set->motor_5_position = 0.0f;
			arm_control_set->motor_6_position = 5.085f;
			if(move_calculate_flag == 20)
			{
				picture_position_flag = PICTURE_POSITION_DOWN;
				rotete_1_flag = ROTETA_RETURN_POSITION_2;
				arm_move_flag = 0;
				move_calculate_flag = 0;
			}
		}
			else if(rotete_1_flag==ROTETA_RETURN_POSITION_2)
		{
			arm_zero_control_set->arm_motor.angle_set = 0.0f;
			arm_control_set->motor_2_position = -1.57f;
			arm_control_set->motor_3_position = -3.0f;
			arm_control_set->motor_4_position = -0.0f;
			arm_control_set->motor_5_position = 0.0f;
			arm_control_set->motor_6_position = 5.085f;
			if(move_calculate_flag == 20)
			{
				rotete_1_flag = INIT_POSITION_ROTATE_1;
				arm_move_flag = 0;
				move_calculate_flag = 0;
			}
		}
//		
//		
//		
		if(zero_exchange_flag == EXCHANGE_LIFT_1_POSITION)
		{
			arm_zero_control_set->arm_motor.angle_set = 0.000f;
			arm_control_set->motor_2_position = -1.57f;
			arm_control_set->motor_3_position =  -3.10f;
			arm_control_set->motor_4_position = -0.047f;
			arm_control_set->motor_5_position = -0.0f;
			arm_control_set->motor_6_position = 0.5954f;
			if(move_calculate_flag == 2)
			{
				zero_exchange_flag = EXCHANGE_LIFT_2_POSITION;
				arm_move_flag = 0;
				move_calculate_flag = 0;
			}
		}
		else if(zero_exchange_flag == EXCHANGE_LIFT_2_POSITION)
		{
			arm_zero_control_set->arm_motor.angle_set = 4.16f;
			arm_control_set->motor_2_position = -1.57f;
			arm_control_set->motor_3_position = -3.10f;
			arm_control_set->motor_4_position =	0.00f;
			arm_control_set->motor_5_position = -0.000f;
			arm_control_set->motor_6_position = -0.626f+3.14f;
			if(move_calculate_flag == 2)
			{
				zero_exchange_flag = EXCHANGE_CHATH_TARGET;
				arm_move_flag = 0;
				move_calculate_flag = 0;
			}
		}
		else if(zero_exchange_flag == EXCHANGE_CHATH_TARGET)
		{
			arm_zero_control_set->arm_motor.angle_set = 4.1615f;
			arm_control_set->motor_2_position = -1.333f;
			arm_control_set->motor_3_position = -0.993f;
			arm_control_set->motor_4_position = 0.0f;
			arm_control_set->motor_5_position = -1.395f;
			arm_control_set->motor_6_position = 3.44f;
			if(move_calculate_flag == 50)
			{
				sucker_flag = OPEN;
			}
			if(move_calculate_flag == 70)
			{
				zero_exchange_flag = EXCHANGE_LIFT_3_POSITION;
				arm_move_flag = 0;
				move_calculate_flag = 0;
			}
		}
		else if(zero_exchange_flag == EXCHANGE_LIFT_3_POSITION)
		{
			arm_zero_control_set->arm_motor.angle_set =  4.1615f;
			arm_control_set->motor_2_position = -1.1f;
			arm_control_set->motor_3_position = -1.80f;
			arm_control_set->motor_4_position =	0.00f;
			arm_control_set->motor_5_position = -0.000f;
			arm_control_set->motor_6_position =	5.085f;
			if(move_calculate_flag == 2)
			{
				zero_exchange_flag = EXCHANGE_LIFT_4_POSITION;
				arm_move_flag = 0;
				move_calculate_flag = 0;
			}
		}
		else if(zero_exchange_flag == EXCHANGE_LIFT_4_POSITION)
		{
			arm_zero_control_set->arm_motor.angle_set =  1.57f;
			arm_control_set->motor_2_position = -1.5f;
			arm_control_set->motor_3_position = -3.10f;
			arm_control_set->motor_4_position =	0.00f;
			arm_control_set->motor_5_position = -0.000f;
			arm_control_set->motor_6_position =	5.085f;
			if(move_calculate_flag == 2)
			{
				zero_exchange_flag = EXCHANGE_PREPARE_POSITION;
				arm_move_flag = 0;
				move_calculate_flag = 0;
			}
		}
		else if(zero_exchange_flag == EXCHANGE_PREPARE_POSITION)
		{
			arm_zero_control_set->arm_motor.angle_set = 1.57f;
			arm_control_set->motor_2_position = -1.2f;
			arm_control_set->motor_3_position =  -1.570f;
			arm_control_set->motor_4_position = -0.347f;
			arm_control_set->motor_5_position = -0.1f;
			arm_control_set->motor_6_position = 4.185f;
			if(move_calculate_flag == 2)
			{
				picture_position_flag = PICTURE_POSITION_UP;
				zero_exchange_flag = EXCHANGE_POSITION;
				arm_move_flag = 0;
				move_calculate_flag = 0;
			}
		}
		else if(zero_exchange_flag == EXCHANGE_POSITION)
		{
			arm_zero_control_set->arm_motor.angle_set = 1.57f;
			arm_control_set->motor_2_position = -1.20f;
			arm_control_set->motor_3_position =  -1.57f;
			arm_control_set->motor_4_position = -0.0f;
			arm_control_set->motor_5_position = -0.1f;
			arm_control_set->motor_6_position = 4.185f;
//			if(move_calculate_flag == 1000)
//			{
//				sucker_flag = CLOSE;
//			}
//			if(move_calculate_flag == 5000)
//			{
//				zero_exchange_flag = INIT_POSITION_EXCHANGE;
//				arm_move_flag = 0;
//				move_calculate_flag = 0;
//			}
		}
		else if(zero_exchange_flag == PREPARE_RETURN_POSITION)
		{
			arm_zero_control_set->arm_motor.angle_set = 1.57f;
			arm_control_set->motor_2_position = -1.57f;
			arm_control_set->motor_3_position =  -3.010f;
			arm_control_set->motor_4_position = -0.0f;
			arm_control_set->motor_5_position = -0.0f;
			arm_control_set->motor_6_position = 4.185f;
			if(move_calculate_flag == 2)
			{
				zero_exchange_flag = EXCHANGE_RETURN_POSITION;
				arm_move_flag = 0;
				move_calculate_flag = 0;
			}
		}
		else if(zero_exchange_flag == PREPARE_RETURN_POSITION)
		{
			arm_zero_control_set->arm_motor.angle_set = 0.0f;
			arm_control_set->motor_2_position = -1.57f;
			arm_control_set->motor_3_position =  -3.010f;
			arm_control_set->motor_4_position = -0.0f;
			arm_control_set->motor_5_position = -0.0f;
			arm_control_set->motor_6_position = 4.185f;
			if(move_calculate_flag == 2)
			{
				zero_exchange_flag = INIT_POSITION_EXCHANGE;
				arm_move_flag = 0;
				move_calculate_flag = 0;
			}
		}
		
		if(observe_flag == 1)
		{
			picture_position_flag = PICTURE_POSITION_UP;
			arm_zero_control_set->arm_motor.angle_set = 0.0f;
			arm_control_set->motor_2_position = -1.57f;
			arm_control_set->motor_3_position =  -3.1f;
			arm_control_set->motor_4_position = -0.0f;
			arm_control_set->motor_5_position = -0.0f;
			arm_control_set->motor_6_position = 0.44f;
			if(move_calculate_flag == 2)
			{
				observe_flag = 2;
				arm_move_flag = 0;
				move_calculate_flag = 0;
			}
		}
		else if(observe_flag == 2)
		{
			arm_zero_control_set->arm_motor.angle_set = 2.15f;
			arm_control_set->motor_2_position = -1.57f;
			arm_control_set->motor_3_position =  -3.1f;
			arm_control_set->motor_4_position = -0.0f;
			arm_control_set->motor_5_position = -0.0f;
			arm_control_set->motor_6_position = 0.44f;
			if(move_calculate_flag == 2)
			{
				observe_flag = 3;
				arm_move_flag = 0;
				move_calculate_flag = 0;
			}
		}
		else if(observe_flag == 3)
		{
			arm_zero_control_set->arm_motor.angle_set = 2.15f;
			arm_control_set->motor_2_position = -0.76f;
			arm_control_set->motor_3_position =  -2.43f;
			arm_control_set->motor_4_position = -0.36f;
			arm_control_set->motor_5_position = -1.65f;
			arm_control_set->motor_6_position = 0.44f;
		}
		else if(observe_flag == 4)
		{
			arm_zero_control_set->arm_motor.angle_set =2.15f;
			arm_control_set->motor_2_position = -1.57f;
			arm_control_set->motor_3_position =  -3.1f;
			arm_control_set->motor_4_position = -0.0f;
			arm_control_set->motor_5_position = -0.0f;
			arm_control_set->motor_6_position = 0.44f;
			if(move_calculate_flag == 20)
			{
				observe_flag = 5;
				arm_move_flag = 0;
				move_calculate_flag = 0;
			}
		}
		else if(observe_flag == 5)
		{
			arm_zero_control_set->arm_motor.angle_set =0.0f;
			arm_control_set->motor_2_position = -1.57f;
			arm_control_set->motor_3_position =  -3.1f;
			arm_control_set->motor_4_position = -0.0f;
			arm_control_set->motor_5_position = -0.0f;
			arm_control_set->motor_6_position = 0.44f;
			if(move_calculate_flag == 2)
			{
				observe_flag = 0;
				arm_move_flag = 0;
				move_calculate_flag = 0;
			}
		}
		
		if(get_ore_flag == PREPARE_GET_POSITION_1)
		{
			arm_zero_control_set->arm_motor.angle_set = 0.000f;
			arm_control_set->motor_2_position = -1.57f;
			arm_control_set->motor_3_position =  -3.10f;
			arm_control_set->motor_4_position = -0.047f;
			arm_control_set->motor_5_position = -0.0f;
			arm_control_set->motor_6_position = 0.5954f;
			if(move_calculate_flag == 2)
			{
				get_ore_flag = PREPARE_GET_POSITION_2;
				arm_move_flag = 0;
				move_calculate_flag = 0;
			}
		}
		else if(get_ore_flag == PREPARE_GET_POSITION_2)
		{
			arm_zero_control_set->arm_motor.angle_set = 4.16f;
			arm_control_set->motor_2_position = -1.57f;
			arm_control_set->motor_3_position = -3.10f;
			arm_control_set->motor_4_position =	0.00f;
			arm_control_set->motor_5_position = -0.000f;
			arm_control_set->motor_6_position = -0.626f+3.14f;
			if(move_calculate_flag == 2)
			{
				get_ore_flag = GET_ORE_POSITION;
				arm_move_flag = 0;
				move_calculate_flag = 0;
			}
		}
		else if(get_ore_flag == GET_ORE_POSITION)
		{
			arm_zero_control_set->arm_motor.angle_set = 4.1615f;
			arm_control_set->motor_2_position = -1.333f;
			arm_control_set->motor_3_position = -0.993f;
			arm_control_set->motor_4_position = 0.0f;
			arm_control_set->motor_5_position = -1.395f;
			arm_control_set->motor_6_position = 3.44f;
			if(move_calculate_flag == 50)
			{
				sucker_flag = OPEN;
			}
			if(move_calculate_flag == 70)
			{
				get_ore_flag = GET_LIFT_UP;
				arm_move_flag = 0;
				move_calculate_flag = 0;
			}
		}
		else if(get_ore_flag == GET_LIFT_UP)
		{
			arm_zero_control_set->arm_motor.angle_set = 4.1615f;
			arm_control_set->motor_2_position = -1.57f;
			arm_control_set->motor_3_position = -3.10f;
			arm_control_set->motor_4_position = 0.0f;
			arm_control_set->motor_5_position = -0.0f;
			arm_control_set->motor_6_position = 3.44f;
		}
//		else
//		{
//			arm_zero_control_set->arm_motor.angle_set = 0.0f;
//			arm_control_set->motor_2_position = 0.0f;
//			arm_control_set->motor_3_position = 0.0f;
//			arm_control_set->motor_4_position = 0.0f;
//			arm_control_set->motor_5_position = 0.0f;
//			arm_control_set->motor_6_position = 0.0f;
//		}
		if(catch_silver_flag == INIT_POSITION &&  catch_silver_1_flag == INIT_POSITION_1 && rotete_1_flag == INIT_POSITION_ROTATE_1 && zero_exchange_flag == INIT_POSITION_EXCHANGE &&observe_flag == 0&&get_ore_flag == INIT_POSITION_GET)
		{
			picture_position_flag = PICTURE_POSITION_DOWN;
			arm_zero_control_set->arm_motor.angle_set = 0.0f;
			arm_control_set->motor_2_position = 0.0f;
			arm_control_set->motor_3_position = 0.0f;
			arm_control_set->motor_4_position = 0.0f;
			arm_control_set->motor_5_position = 0.0f;
			arm_control_set->motor_6_position = 0.0f;
		}
}
 



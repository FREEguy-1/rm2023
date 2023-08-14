#include "chassis_behaviour.h"
#include "cmsis_os.h"
#include "chassis_task.h"
#include "arm_math.h"
#include "pc_info_task.h"
#include "air_control_task.h"

static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);
static void pc_control(fp32 *vx_set, fp32 *vy_set,  fp32 *wz_set,chassis_move_t *chassis_move_rc_to_vector);


uint8_t chassis_behaviour_flag; 
chassis_behaviour_e chassis_behaviour_mode = CHASSIS_ZERO_FORCE;


void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
	if (chassis_move_mode == NULL)
	{
		return;
	}

	if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
	{
//		if(pc_receive_msg.rx_data.cameramode == 3 && pc_send_msg.tx_data.cameraType == 3)
//		{
//			chassis_behaviour_mode = PC_CONTROL;
//		}
//		else
//		{
			chassis_behaviour_mode = CHASSIS_MOVE;
//		}    
	}   
	else if(switch_is_up(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
	{
		chassis_behaviour_mode = CHASSIS_MOVE;
	}
	else if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
	{
		chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
	}
	
	if (chassis_behaviour_mode == CHASSIS_MOVE)
	{
		chassis_behaviour_flag = CHASSIS_MOVE;
	}
	else if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
	{
		chassis_behaviour_flag = CHASSIS_ZERO_FORCE;
	}
	else if (chassis_behaviour_mode == PC_CONTROL)
	{
		chassis_behaviour_flag = PC_CONTROL;
	}
}




void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
	{
			return;
	}

	if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
	{
		chassis_zero_force_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
	}
	else if (chassis_behaviour_mode == CHASSIS_MOVE)
	{
		chassis_move_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
	}
	else if (chassis_behaviour_mode == PC_CONTROL)
	{
		pc_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
	}
}




static void chassis_zero_force_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
	{
		return;
	}
	*vx_set = 0.0f;
	*vy_set = 0.0f;
	*wz_set = 0.0f;
}




static void chassis_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
	{
		return;
	}
	chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
	if(mode_control_flag !=  SECOND_MODE&&chassis_move.chassis_RC->key.CTRL != 1)
	{
		if(chassis_move_rc_to_vector->chassis_RC->key.A)
		{
			if(chassis_move_rc_to_vector->chassis_RC->key.SHIFT)
			{	
				*wz_set = NORMAL_MAX_CHASSIS_SPEED_Z;
			}
			else
			{
				*wz_set = SLOW_CHASSIS_SPEED_Z;
			}
		}
		else if(chassis_move_rc_to_vector->chassis_RC->key.D)
		{
			if(chassis_move_rc_to_vector->chassis_RC->key.SHIFT)
			{	
				*wz_set = -NORMAL_MAX_CHASSIS_SPEED_Z;
			}
			else
			{
				*wz_set = -SLOW_CHASSIS_SPEED_Z;
			}
		}
	}
	if(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL] != 0)
	{
		*wz_set = -CHASSIS_WZ_RC_SEN * chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL];
	}
//	else if(chassis_wz_move_flag_2 != 0) 
//	{
//			*wz_set =-CHASSIS_MOUSE_CONTROL_WZ * chassis_wz_move_flag_2;
//	}
}

static void pc_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_pc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || chassis_move_pc_to_vector == NULL)
    {
        return;
    }
		chassis_pc_to_control_vector(vx_set, vy_set, wz_set, chassis_move_pc_to_vector);
}


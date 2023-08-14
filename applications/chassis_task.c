/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             底盘控制任务

  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "pc_info_task.h"
#include "air_control_task.h"



#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

		
static void chassis_init(chassis_move_t *chassis_move_init);
static void chassis_set_mode(chassis_move_t *chassis_move_mode);
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);
//static void J_scope_gimbal_test(void);

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif


chassis_move_t chassis_move;


void chassis_task(void const *pvParameters)
{
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    chassis_init(&chassis_move);
    while (1)
    {
//		J_scope_gimbal_test();
		chassis_set_mode(&chassis_move);
		chassis_feedback_update(&chassis_move);
		chassis_set_contorl(&chassis_move);
		chassis_control_loop(&chassis_move);
		CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
										chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
		vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}



/*底盘初始化函数，
  分别生成速度环PID数组与视觉控制PID数组，
  将四个轮子的第一个PID环设定为速度环，
  接收轮子can通信发回数组的初始状态*/
static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }
    const static fp32 motor_speed_pid[4][3] = { {M3508_MOTOR0_SPEED_PID_KP, M3508_MOTOR0_SPEED_PID_KI, M3508_MOTOR0_SPEED_PID_KD},
																								{M3508_MOTOR1_SPEED_PID_KP, M3508_MOTOR1_SPEED_PID_KI, M3508_MOTOR1_SPEED_PID_KD},
																								{M3508_MOTOR2_SPEED_PID_KP, M3508_MOTOR2_SPEED_PID_KI, M3508_MOTOR2_SPEED_PID_KD},
																								{M3508_MOTOR3_SPEED_PID_KP, M3508_MOTOR3_SPEED_PID_KI, M3508_MOTOR3_SPEED_PID_KD}};
    const static fp32 pc_ore_control_pid[3]={PC_ORE_CONTROL_KP,PC_ORE_CONTROL_KI,PC_ORE_CONTROL_KD};    
    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
    chassis_move_init->chassis_RC = get_remote_control_point();
    for (uint8_t i = 0; i < 4; i++)
    {
			chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
			PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid[i], M3508_MOTOR_SPEED_PID_MAX_OUT, M3508_MOTOR_SPEED_PID_MAX_IOUT);
    }
		PID_init(&chassis_move_init->pc_ore_control_pid , PID_POSITION, pc_ore_control_pid, PC_ORE_CONTROL_PID_MAX_OUT, PC_ORE_CONTROL_PID_MAX_IOUT);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);
    chassis_feedback_update(chassis_move_init);
}


/*此处跳转到chassis_behaviour.c文件中的底盘运动模式控制中调整底盘的运动模式*/
static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
	if (chassis_move_mode == NULL)
	{
			return;
	}
	chassis_behaviour_mode_set(chassis_move_mode);
}

/*此处分析轮子电机的反馈数据，将轮子的角速度转为线速度并计算轮子转动的角度，
  另外通过麦轮正解算算出车速（此处的角度不太准且，建议另外用编码器判断电机角度）*/
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }
    for (uint8_t i = 0; i < 4; i++)
    {
			chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
			chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
    }
    chassis_move_update->vx = (-chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    chassis_move_update->vy = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    chassis_move_update->wz = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;
}




/*此处使用遥控器数据来控制车辆各方向移动速度（遥控器包括大疆遥控器、键盘以及鼠标）*/
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
	{
			return;
	}

	int16_t vx_channel, vy_channel;
	fp32 vx_set_channel, vy_set_channel;
	rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
	rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);
	vx_set_channel = vx_channel * -CHASSIS_VX_RC_SEN;
	vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;
	if(mode_control_flag != SECOND_MODE && chassis_move.chassis_RC->key.CTRL != 1)
	{
		if(chassis_move_rc_to_vector->chassis_RC->key.SHIFT)
		{
				if (chassis_move_rc_to_vector->chassis_RC->key.W)
				{	
						vx_set_channel = -NORMAL_MAX_CHASSIS_SPEED_X;
				}
				else if (chassis_move_rc_to_vector->chassis_RC->key.S)
				{
						vx_set_channel = NORMAL_MAX_CHASSIS_SPEED_X;
				}
				if (chassis_move_rc_to_vector->chassis_RC->key.Q)
				{
						vy_set_channel = NORMAL_MAX_CHASSIS_SPEED_Y;
				}
				else if (chassis_move_rc_to_vector->chassis_RC->key.E)
				{
						vy_set_channel = -NORMAL_MAX_CHASSIS_SPEED_Y;
				}

		}
		else
		{
				if (chassis_move_rc_to_vector->chassis_RC->key.W)
				{	
						vx_set_channel = -SLOW_CHASSIS_SPEED_X;
				}
				else if (chassis_move_rc_to_vector->chassis_RC->key.S)
				{
						vx_set_channel = SLOW_CHASSIS_SPEED_X;
				}

				if (chassis_move_rc_to_vector->chassis_RC->key.Q)
				{
						vy_set_channel = SLOW_CHASSIS_SPEED_Y;
				}
				else if (chassis_move_rc_to_vector->chassis_RC->key.E)
				{
						vy_set_channel = -SLOW_CHASSIS_SPEED_Y;
				}
			
		}
	}
	first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
	first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);//这里过两个低通滤波器
	if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
	{
			chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
	}

	if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
	{
			chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
	}//当遥控器拨杆数值小于死区时车辆的设定值速度直接为零，避免遥控器误差导致车停不下来

	*vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
	*vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;//将遥控数据计算后的值作为车速设定值
}



/*此处可将地盘控制权交给视觉自动控制，根据视觉返回的位置信息得到车辆的移动数据*/
void chassis_pc_to_control_vector(fp32 *vx_set, fp32 *vy_set,fp32 *wz_set, chassis_move_t *chassis_move_pc_to_vector)
{
	if (chassis_move_pc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
	{
			return;
	}
//	if(pc_receive_msg.rx_data.cameramode == 3)
//	{
//		*vx_set =  PID_calc(&chassis_move_pc_to_vector->pc_ore_control_pid, pc_receive_msg.rx_data.x, 0);
//		if(*vx_set > PC_MAX_CHASSIS_SPEED_X)
//		{
//			*vx_set = PC_MAX_CHASSIS_SPEED_X; 
//		}
//		else if(*vx_set < -PC_MAX_CHASSIS_SPEED_X)
//		{
//			*vx_set  = -PC_MAX_CHASSIS_SPEED_X;
//		}
//		*wz_set = 0;
//		*vy_set = 0;
//	}
//	else if(pc_receive_msg.rx_data.cameramode == 6)
//	{
//		*vx_set =  PID_calc(&chassis_move_pc_to_vector->pc_ore_control_pid, pc_receive_msg.rx_data.x, 0);
//		if(*vx_set > PC_MAX_CHASSIS_SPEED_X)
//		{
//			*vx_set = PC_MAX_CHASSIS_SPEED_X; 
//		}
//		else if(*vx_set < -PC_MAX_CHASSIS_SPEED_X)
//		{
//			*vx_set  = -PC_MAX_CHASSIS_SPEED_X;
//		}
//		*wz_set = 0;
//		*vy_set = 0;
//	}
}


/*此处跳转到chassis_behaviour.c文件中，
	结合底盘不同的运动模式以及根据遥控器计算的数据来计算出车速实际设定值*/
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
	if (chassis_move_control == NULL)
	{
			return;
	}
	fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
	chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);
	chassis_move_control->wz_set = angle_set;
	chassis_move_control->vx_set = fp32_constrain(vx_set, -NORMAL_MAX_CHASSIS_SPEED_X, NORMAL_MAX_CHASSIS_SPEED_X);
	chassis_move_control->vy_set = fp32_constrain(vy_set, -NORMAL_MAX_CHASSIS_SPEED_Y, NORMAL_MAX_CHASSIS_SPEED_Y);//通过两个带通滤波器，限制车速
}



/*此处根据射速的设定值计算每个轮子的转速设定值，
	涉及麦轮逆结算，麦轮结构的车可以实现x,y,wz三个方向运动合成，
	因此此处可以实现一遍直走一边拐弯*/
static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
    wheel_speed[0] = -vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[1] = vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[2] = vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[3] = -vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
}

/*此处利用速度环pid计算电机需要的电流大小，并在上方任务函数中发送电流*/
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
	fp32 max_vector = 0.0f, vector_rate = 0.0f;
	fp32 temp = 0.0f;
	fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
	uint8_t i = 0;

	chassis_vector_to_mecanum_wheel_speed(-chassis_move_control_loop->vx_set,
																				chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set, wheel_speed);

	for (i = 0; i < 4; i++)
	{
		chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
		temp = fabs(chassis_move_control_loop->motor_chassis[i].speed_set);
		if (max_vector < temp)
		{
				max_vector = temp;
		}
	}
	if (max_vector > MAX_WHEEL_SPEED)
	{
		vector_rate = MAX_WHEEL_SPEED / max_vector;
		for (i = 0; i < 4; i++)
		{
				chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
		}
	}
	for (i = 0; i < 4; i++)
	{
		PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
	}

	for (i = 0; i < 4; i++)
	{
		chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
	}
}

/*   J-scope 测试   */
//int32_t speed_0;
//int32_t speed_1;
//int32_t speed_2;
//int32_t speed_3;
//int32_t speed_set_0;
//int32_t speed_set_1;
//int32_t speed_set_2;
//int32_t speed_set_3;
//static void J_scope_gimbal_test(void)
//{
//	speed_0=1000*chassis_move.motor_chassis[0].speed;
//	speed_set_0=1000*chassis_move.motor_chassis[0].speed_set;
//	speed_1=1000*chassis_move.motor_chassis[1].speed;
//	speed_set_1=1000*chassis_move.motor_chassis[1].speed_set;
//	speed_2=1000*chassis_move.motor_chassis[2].speed;
//	speed_set_1=1000*chassis_move.motor_chassis[2].speed_set;
//	speed_3=1000*chassis_move.motor_chassis[3].speed;
//	speed_set_3=1000*chassis_move.motor_chassis[3].speed_set;
//}

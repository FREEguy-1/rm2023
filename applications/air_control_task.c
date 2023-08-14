#include "air_control_task.h"
#include "keyboard.h"
#include "remote_control.h"
#include "chassis_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "tim.h"
#include "A_communicate_task.h"
#include "pc_info_task.h"


static void air_control_init(all_key_t *air_control_init); //气缸与真空泵、上层供电初始化函数
static void air_control_key_check(all_key_t *air_control_key_check); //气缸与真空泵、上层供电按键触发检查
static void air_control_flags_set(all_key_t *air_control_flags_set); // 气缸与真空泵、上层供电标志位设置
static void air_task_control(void); //通过标志位更改io口电平，实现气缸与真空泵、上层供电控制
static void mode_control_set(void); //通过遥控器数值判断控制模式


chassis_clock_e chassis_clock_flag;  //底盘锁定气缸控制标志位
picture_position_e picture_position_flag; //图传位置标志位
sucker_e sucker_flag;//真空泵工作标志位
send_power_e power_flag; //供电标志位
mode_e mode_control_flag;//模式设置标志位
//servo_position_e servo_position_flag; //舵机位置标志位
uint16_t servo_1_pwm_out;
uint16_t servo_2_pwm_out;
fp32 servo_1_temp;
fp32 servo_2_temp;

void air_control_task(void const * argument)
{
	air_control_init(&all_key); //开机时先对几个气缸进行初始化
	while(1) 
	{
		mode_control_set();//根据遥控器数据设定控制模式
		air_control_key_check(&all_key);  // 循环检查相关按键状态
		air_control_flags_set(&all_key);  // 通过按键来控制气缸标志位
		air_task_control();  //根据标志位改变气缸对应io口电平
		osDelay(10);
	}
}


/*控制按键设定为模式一下的Z,X,C,初始状态为未锁定、真空泵关闭、图传位置为下*/
void air_control_init(all_key_t *air_control_init)
{
//	key_init(&air_control_init->first_mode_key.chassis_clock_key, Z);
	key_init(&air_control_init->first_mode_key.picture_position_key,F);
	key_init(&air_control_init->first_mode_key.sucker_key, R);
//	key_init(&air_control_init->first_mode_key.send_power_key,V);
//	key_init(&air_control_init->first_mode_key.servo_key,B);
	key_init(&air_control_init->mode_control_key, CTRL);
	
	sucker_flag = CLOSE;
	chassis_clock_flag = CHASSIS_BE_UNCLOCK;
	picture_position_flag = PICTURE_POSITION_DOWN;
	power_flag = POWER_CLOSE;
//	mode_control_flag = FIRST_MODE;
//	servo_position_flag = SERVO_POSITION_INIT;
	servo_1_temp = 1500.0f;
	servo_2_temp = 1500.0f;
	servo_1_pwm_out = 1500;
	servo_2_pwm_out = 1500;
	osDelay(3000);
}


/*拨杆1中间时为模式一，靠上时为模式二，靠下时为模式三*/
void mode_control_set(void)
{
	if(switch_is_mid(chassis_move.chassis_RC->rc.s[1]))
	{
		mode_control_flag = FIRST_MODE;
	}
	else if(switch_is_up(chassis_move.chassis_RC->rc.s[1]))
	{
		mode_control_flag = SECOND_MODE;
	}
	else if(switch_is_down(chassis_move.chassis_RC->rc.s[1]))
	{
		mode_control_flag = THIRD_MODE;
	}
}

/*检查各控制按键是否触发*/
void air_control_key_check(all_key_t *air_control_key_check)
{
	if(mode_control_flag == SECOND_MODE)
	{
//		key_itself_press_num(&(air_control_key_check->first_mode_key.chassis_clock_key), 2);
		key_itself_press_num(&(air_control_key_check->first_mode_key.picture_position_key), 2);
		key_itself_press_num(&(air_control_key_check->first_mode_key.sucker_key), 2);
//		key_itself_press_num(&(air_control_key_check->first_mode_key.send_power_key), 2);
//		key_itself_press_num(&(air_control_key_check->first_mode_key.servo_key), 3);
	}
	key_itself_press_num(&(air_control_key_check->first_mode_key.sucker_key), 2);
}

/*每当按键按下一次后标志位取反*/
void air_control_flags_set(all_key_t *air_control_flags_set)
{
	if(rx_motor_position[0]<0.1f && rx_motor_position[1] < 0.1f && rx_motor_position[2]<0.1f&&rx_motor_position[3]<0.1f
		&&rx_motor_position[0]>-0.1f && rx_motor_position[1] > -0.1f && rx_motor_position[2]>-0.1f&&rx_motor_position[3]>-0.1f)
	{
		chassis_clock_flag = CHASSIS_BE_UNCLOCK;
		osDelay(1);
	}
	else
	{
		chassis_clock_flag = CHASSIS_BE_CLOCK;
		osDelay(1);
	}
	/*底盘锁定气缸标志位设置*/
//	if(air_control_flags_set->first_mode_key.chassis_clock_key.itself.last_mode != air_control_flags_set->first_mode_key.chassis_clock_key.itself.mode)
//	{
//		if(chassis_clock_flag == CHASSIS_BE_UNCLOCK)
//		{
//			chassis_clock_flag = CHASSIS_BE_CLOCK;
//		}
//		else
//		{
//			chassis_clock_flag = CHASSIS_BE_UNCLOCK;
//		}
//	}
	
	/*图传位置气缸标志位设置*/
	if(air_control_flags_set->first_mode_key.picture_position_key.itself.last_mode != air_control_flags_set->first_mode_key.picture_position_key.itself.mode)
	{
		if(picture_position_flag == PICTURE_POSITION_DOWN)
		{
			picture_position_flag = PICTURE_POSITION_UP;
		}
		else
		{
			picture_position_flag = PICTURE_POSITION_DOWN;
		}
	}
	
	/*真空泵标志位设置*/
	if(air_control_flags_set->first_mode_key.sucker_key.itself.last_mode != air_control_flags_set->first_mode_key.sucker_key.itself.mode)
	{
		if(sucker_flag == CLOSE)
		{
			sucker_flag = OPEN;
		}
		else
		{
			sucker_flag = CLOSE;
		}
	}
	
	if(pc_receive_msg.rx_data.pc_receive_flag == 2)
	{
		sucker_flag =	OPEN;
	}
	else if(pc_receive_msg.rx_data.pc_receive_flag == 1)
	{
		sucker_flag =	CLOSE;
	}
	
	/*上层供电标志位设置*/
	if(air_control_flags_set->first_mode_key.send_power_key.itself.last_mode != air_control_flags_set->first_mode_key.send_power_key.itself.mode)
	{
		if(power_flag == CLOSE)
		{
			power_flag = POWER_OPEN;
		}
		else
		{
			power_flag = POWER_CLOSE;
		}
	}
	servo_1_temp += (fp32)chassis_move.chassis_RC->mouse.x*0.5f;
	servo_2_temp += (fp32)chassis_move.chassis_RC->mouse.y*0.5f;
	servo_1_pwm_out = (uint16_t)servo_1_temp;
	servo_2_pwm_out = (uint16_t)servo_2_temp;
	if(servo_1_pwm_out>2500)
	{
		servo_1_pwm_out = 2500;
	}
	else if(servo_1_pwm_out < 500)
	{
		servo_1_pwm_out = 500;
	}
	if(servo_2_pwm_out>2500)
	{
		servo_2_pwm_out = 2500;
	}
	else if(servo_2_pwm_out < 500)
	{
		servo_2_pwm_out = 500;
	}
	
	/*舵机位置标志位设置*/
//	if(air_control_flags_set->first_mode_key.servo_key.itself.mode == 0)
//	{
//		servo_position_flag = SERVO_POSITION_INIT;
//	}
//	else if(air_control_flags_set->first_mode_key.servo_key.itself.mode == 1)
//	{
//		servo_position_flag = SERVO_POSITION_DISPLAY;
//	}
//	else if(air_control_flags_set->first_mode_key.servo_key.itself.mode == 2)
//	{
//		servo_position_flag = SERVO_POSITION_RETURN;
//	}
}	


/*实现控制*/
void air_task_control(void)
{
	if(chassis_clock_flag == CHASSIS_BE_CLOCK)
	{
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1,GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1,GPIO_PIN_RESET);
	}
	
	if(picture_position_flag == PICTURE_POSITION_UP)
	{
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_SET);
	}
	
	if(sucker_flag == OPEN)
	{
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_RESET);
	}
	
	if(power_flag == OPEN)
	{
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET);
	}
	
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, servo_1_pwm_out);    //PITCH， 550
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, servo_2_pwm_out);    //YAW  2200
	
}

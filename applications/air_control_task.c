#include "air_control_task.h"
#include "keyboard.h"
#include "remote_control.h"
#include "chassis_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "tim.h"
#include "A_communicate_task.h"
#include "pc_info_task.h"


static void air_control_init(all_key_t *air_control_init); //��������ձá��ϲ㹩���ʼ������
static void air_control_key_check(all_key_t *air_control_key_check); //��������ձá��ϲ㹩�簴���������
static void air_control_flags_set(all_key_t *air_control_flags_set); // ��������ձá��ϲ㹩���־λ����
static void air_task_control(void); //ͨ����־λ����io�ڵ�ƽ��ʵ����������ձá��ϲ㹩�����
static void mode_control_set(void); //ͨ��ң������ֵ�жϿ���ģʽ


chassis_clock_e chassis_clock_flag;  //�����������׿��Ʊ�־λ
picture_position_e picture_position_flag; //ͼ��λ�ñ�־λ
sucker_e sucker_flag;//��ձù�����־λ
send_power_e power_flag; //�����־λ
mode_e mode_control_flag;//ģʽ���ñ�־λ
//servo_position_e servo_position_flag; //���λ�ñ�־λ
uint16_t servo_1_pwm_out;
uint16_t servo_2_pwm_out;
fp32 servo_1_temp;
fp32 servo_2_temp;

void air_control_task(void const * argument)
{
	air_control_init(&all_key); //����ʱ�ȶԼ������׽��г�ʼ��
	while(1) 
	{
		mode_control_set();//����ң���������趨����ģʽ
		air_control_key_check(&all_key);  // ѭ�������ذ���״̬
		air_control_flags_set(&all_key);  // ͨ���������������ױ�־λ
		air_task_control();  //���ݱ�־λ�ı����׶�Ӧio�ڵ�ƽ
		osDelay(10);
	}
}


/*���ư����趨Ϊģʽһ�µ�Z,X,C,��ʼ״̬Ϊδ��������ձùرա�ͼ��λ��Ϊ��*/
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


/*����1�м�ʱΪģʽһ������ʱΪģʽ��������ʱΪģʽ��*/
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

/*�������ư����Ƿ񴥷�*/
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

/*ÿ����������һ�κ��־λȡ��*/
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
	/*�����������ױ�־λ����*/
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
	
	/*ͼ��λ�����ױ�־λ����*/
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
	
	/*��ձñ�־λ����*/
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
	
	/*�ϲ㹩���־λ����*/
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
	
	/*���λ�ñ�־λ����*/
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


/*ʵ�ֿ���*/
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
	
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, servo_1_pwm_out);    //PITCH�� 550
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, servo_2_pwm_out);    //YAW  2200
	
}

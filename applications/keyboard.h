#ifndef KEY_H
#define KEY_H
#include "struct_typedef.h"

typedef enum
{
		W,
		S,
		A,
		D,
		SHIFT,
		CTRL,
		Q,
		E,
		R,
		F,
		G,
		Z,
		X,
		C,
		V,
		B,
		PR,
		PL,

}key_e;

typedef enum
{
	FIRST_MODE,
	SECOND_MODE,
	THIRD_MODE,
}mode_e;

typedef struct
{
	int8_t mode;
	int8_t time;
	int8_t flag;
	int8_t last_mode;
}key_mode_t;

typedef struct
{
	key_e content;
	key_mode_t itself;
}key_t;

typedef struct
{
	key_t mode_control_key;
	key_t p_left; 
	key_t p_right;
	struct
	{
		key_t chassis_clock_key;       
		key_t picture_position_key;       
		key_t sucker_key;    
		key_t send_power_key;     
		key_t servo_key;            
		key_t catch_silver_ore_key;
		key_t catch_silver_ore_key_1;
		key_t rotate_ore_key_1;    
		key_t zero_exchange; 
		key_t observe_move;
		key_t clock_version_position;
		key_t get_ore_position;
		key_t version_move;
		key_t version_move_1;
		key_t go_back;
	}first_mode_key;      
	struct
	{
		key_t lift1_key;   
		key_t lift2_key;   
		key_t sucker_key;  
		key_t grab_key;    
		key_t store1_key;  
		key_t store2_key;  
		key_t rotate_key; 
		key_t stretch_key; 
	}second_mode_key;   
	struct
	{   
		key_t rescue_hand_key; 		
	  key_t rescue_card_key;
		key_t servo_key; 
	  key_t servo_reset_key;
	}third_mode_key;   
}all_key_t;

extern all_key_t all_key;

extern void key_init(key_t *key_init, key_e KEY);
extern void key_itself_press_num(key_t *press_num, int8_t num);  
extern int8_t key_press(key_t *press);     
#endif

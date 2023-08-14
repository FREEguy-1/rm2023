//#include "test_task.h"
//#include "main.h"
//#include "cmsis_os.h"
//#include "bsp_buzzer.h"
//#include "detect_task.h"

////#include "remote_control.h"
////const RC_ctrl_t *test_RC;

//static void buzzer_warn_error(uint8_t num);
//const error_t *error_list_test_local;
//uint8_t error, last_error;

//void test_task(void const * argument)
//{
//	
////	test_RC = get_remote_control_point();
//	
//    static uint8_t error_num;
//    error_list_test_local = get_error_list_point();
//    while(1)
//    {
////        error = 0;
////        //·¢ÏÖ´íÎó
////        for(error_num = 0; error_num < ERROR_LIST_LENGHT; error_num++)
////        {
////            if(error_list_test_local[error_num].error_exist)
////            {
////                error = 1;
////                break;
////            }
////        }
//////				if(switch_is_mid(test_RC->rc.s[1]))
//////				{
//////					buzzer_on(1, 5000);
//////				}
//////				if(switch_is_up(test_RC->rc.s[1]))
//////				{
//////					buzzer_off();
//////				}
////				
////        //Ã»ÓĞ´íÎó, Í£Ö¹·äÃùÆ÷
////        if(error == 0 && last_error != 0)
////        {
////            buzzer_off();
////        }
////        //ÓĞ´íÎó
////        if(error)
////        {
////            buzzer_warn_error(error_num+1);
////        }
////        last_error = error;
//        osDelay(50);
//    }
//}

////¿ØÖÆ·äÃùÆ÷Ïì
////num :·äÃùÆ÷ÏìµÄ´ÎÊı
//static void buzzer_warn_error(uint8_t num)
//{
//		static uint8_t show_num = 0;
//		static uint8_t stop_num = 10;	
//		#if BUZZER_MODE	
//		if(show_num == 0 && stop_num == 0)
//		{
//				show_num = num;
//				stop_num = 10;
//		}
//		else if(show_num == 0)
//		{
//				stop_num--;
//				buzzer_off();
//		}
//		else
//		{
//				static uint8_t tick = 0;
//				tick++;
//				if(tick < 5)
//				{
//					buzzer_off();
//				}
//				else if(tick < 10)
//				{
//					buzzer_on(1, 5000);
//				}
//				else
//				{
//					tick = 0;
//					show_num--;
//				}
//		}
//	#endif
//}

#include "detect_task.h"
#include "math.h"
#include "cmsis_os.h"

static void detect_init(void);
error_t error_list[ ERROR_LIST_LENGHT + 1];

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t detect_task_stack;
#endif

uint32_t system_time;

//�������
void detect_task(void const *pvParameters)
{
    system_time = xTaskGetTickCount();
    detect_init();
    vTaskDelay(DETECT_TASK_INIT_TIME);
    while (1)
    {
        system_time = xTaskGetTickCount();
        for (int i = 0; i < ERROR_LIST_LENGHT; i++)
        {
					error_list[i].error_time = (system_time - error_list[i].new_time);
            //�жϵ���
            if (error_list[i].error_time > error_list[i].set_offline_time)
            {
								error_list[i].error_exist  = 1;
            }
            else
            {
								error_list[i].new_time = xTaskGetTickCount();
                error_list[i].error_exist = 0;
            }
        }
        vTaskDelay(DETECT_CONTROL_TIME);
#if INCLUDE_uxTaskGetStackHighWaterMark
        detect_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}
//��¼ʱ��
void detect_hook(uint8_t toe)
{
    error_list[toe].new_time = xTaskGetTickCount();
}
//�õ������б�
const error_t *get_error_list_point(void)
{
    return error_list;
}
static void detect_init(void)
{
	//��������ʱ�䣬�����ȶ�����ʱ�䣬���ȼ�
	uint16_t set_item[ERROR_LIST_LENGHT]={40, 10, 10, 10, 10, 10};
    for (uint8_t i = 0; i < ERROR_LIST_LENGHT; i++)
    {
        error_list[i].set_offline_time = set_item[i];
        error_list[i].error_exist = 1;
        error_list[i].new_time = xTaskGetTickCount();
    }
}

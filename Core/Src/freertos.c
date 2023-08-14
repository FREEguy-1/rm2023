/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "chassis_task.h"
#include "detect_task.h"
#include "VOFA.h"
#include "pc_info_task.h"
#include "UI_task.h"
#include "referee_usart_task.h"
#include "air_control_task.h"
#include "A_communicate_task.h"
#include "arm_zero_task.h"
#include "arm_control_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId chassisTaskHandle;
//osThreadId detect_handle;
osThreadId VofaTaskHandle;
//osThreadId testHandle;
osThreadId pcinfotaskHandle;
//osThreadId RefereeusartTaskHandle;
//osThreadId UITaskHandle;
osThreadId AircontroltaskHandle;
osThreadId AcommunicatetaskHandle;
osThreadId ArmzerotaskHandle;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId ArmcontroltaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
//	osThreadDef(Mousedelay, mouse_delay_task, osPriorityAboveNormal, 0, 256);
//  MousedelayTaskHandle = osThreadCreate(osThread(Mousedelay), NULL);
	
	osThreadDef(ChassisTask, chassis_task, osPriorityHigh, 0, 512);
  chassisTaskHandle = osThreadCreate(osThread(ChassisTask), NULL);


//  osThreadDef(DETECT, detect_task, osPriorityNormal, 0, 256);
//  detect_handle = osThreadCreate(osThread(DETECT), NULL);
	
//	osThreadDef(test, test_task, osPriorityNormal, 0, 128);
//  testHandle = osThreadCreate(osThread(test), NULL);

//	osThreadDef(VofaTask, vofa_task, osPriorityAboveNormal, 0, 256);
//  VofaTaskHandle = osThreadCreate(osThread(VofaTask), NULL);
	
//	osThreadDef(RefereeusartTask, referee_usart_task, osPriorityAboveNormal, 0, 256);
//  RefereeusartTaskHandle = osThreadCreate(osThread(RefereeusartTask), NULL);
//	
//	osThreadDef(UITask, ui_task, osPriorityAboveNormal, 0, 256);
//  UITaskHandle = osThreadCreate(osThread(UITask), NULL);
	
	osThreadDef(pcinfotask, pc_info_task, osPriorityAboveNormal, 0, 256);
  pcinfotaskHandle = osThreadCreate(osThread(pcinfotask), NULL);
	
	osThreadDef(AircontrolTask, air_control_task, osPriorityNormal, 0, 256);
	AircontroltaskHandle = osThreadCreate(osThread(AircontrolTask), NULL);
	
	osThreadDef(AcommunicateTask, A_communicate_task, osPriorityAboveNormal, 0, 512);
	AcommunicatetaskHandle = osThreadCreate(osThread(AcommunicateTask), NULL);

	osThreadDef(ArmzeroTask, arm_zero_task, osPriorityAboveNormal, 0, 256);
	ArmzerotaskHandle = osThreadCreate(osThread(ArmzeroTask), NULL);
	
	osThreadDef(ArmcontrolTask, arm_control_task, osPriorityNormal, 0, 256);
	ArmcontroltaskHandle = osThreadCreate(osThread(ArmcontrolTask), NULL);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

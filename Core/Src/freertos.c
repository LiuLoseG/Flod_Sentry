/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
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

/*------------------------------栈溢出现场记录--------------------------------*/
/* configCHECK_FOR_STACK_OVERFLOW = 2 时, 内核在每次上下文切换中比对被换出任务栈尾部的
   填充字(0xA5)是否被破坏, 一旦被覆盖就调用 vApplicationStackOverflowHook(见下方 USER CODE 4).
   这些变量只写不读, 供调试器(Ozone/J-Link)在停机后直接查看定责. */
volatile uint32_t           stack_overflow_count     = 0U;   /* 溢出累计次数 */
volatile TaskHandle_t       stack_overflow_task      = NULL; /* 溢出的任务句柄 */
volatile const char        *stack_overflow_task_name = NULL; /* 溢出的任务名 */
volatile uint32_t           stack_overflow_psp       = 0U;   /* 溢出时任务 PSP, 用于定位越界深度 */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 256 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for test */
osThreadId_t testHandle;
uint32_t testBuffer[ 128 ];
osStaticThreadDef_t testControlBlock;
const osThreadAttr_t test_attributes = {
  .name = "test",
  .cb_mem = &testControlBlock,
  .cb_size = sizeof(testControlBlock),
  .stack_mem = &testBuffer[0],
  .stack_size = sizeof(testBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for remote_control */
osThreadId_t remote_controlHandle;
uint32_t remote_controlBuffer[ 512 ];
osStaticThreadDef_t remote_controlControlBlock;
const osThreadAttr_t remote_control_attributes = {
  .name = "remote_control",
  .cb_mem = &remote_controlControlBlock,
  .cb_size = sizeof(remote_controlControlBlock),
  .stack_mem = &remote_controlBuffer[0],
  .stack_size = sizeof(remote_controlBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for decision_center */
osThreadId_t decision_centerHandle;
uint32_t decision_centerBuffer[ 1024 ];
osStaticThreadDef_t decision_centerControlBlock;
const osThreadAttr_t decision_center_attributes = {
  .name = "decision_center",
  .cb_mem = &decision_centerControlBlock,
  .cb_size = sizeof(decision_centerControlBlock),
  .stack_mem = &decision_centerBuffer[0],
  .stack_size = sizeof(decision_centerBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for safe_daemon */
osThreadId_t safe_daemonHandle;
uint32_t safe_daemonBuffer[ 256 ];
osStaticThreadDef_t safe_daemonControlBlock;
const osThreadAttr_t safe_daemon_attributes = {
  .name = "safe_daemon",
  .cb_mem = &safe_daemonControlBlock,
  .cb_size = sizeof(safe_daemonControlBlock),
  .stack_mem = &safe_daemonBuffer[0],
  .stack_size = sizeof(safe_daemonBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ins_task_rtos */
osThreadId_t ins_task_rtosHandle;
uint32_t ins_task_rtosBuffer[ 1024 ];
osStaticThreadDef_t ins_task_rtosControlBlock;
const osThreadAttr_t ins_task_rtos_attributes = {
  .name = "ins_task_rtos",
  .cb_mem = &ins_task_rtosControlBlock,
  .cb_size = sizeof(ins_task_rtosControlBlock),
  .stack_mem = &ins_task_rtosBuffer[0],
  .stack_size = sizeof(ins_task_rtosBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for chassis */
osThreadId_t chassisHandle;
uint32_t chassisBuffer[ 1024 ];
osStaticThreadDef_t chassisControlBlock;
const osThreadAttr_t chassis_attributes = {
  .name = "chassis",
  .cb_mem = &chassisControlBlock,
  .cb_size = sizeof(chassisControlBlock),
  .stack_mem = &chassisBuffer[0],
  .stack_size = sizeof(chassisBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void test_task(void *argument);
void remote_control_task(void *argument);
void decision_center_task(void *argument);
void safe_daemon_task(void *argument);
void imu_task(void *argument);
void chassis_task(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of test */
  testHandle = osThreadNew(test_task, NULL, &test_attributes);

  /* creation of remote_control */
  remote_controlHandle = osThreadNew(remote_control_task, NULL, &remote_control_attributes);

  /* creation of decision_center */
  decision_centerHandle = osThreadNew(decision_center_task, NULL, &decision_center_attributes);

  /* creation of safe_daemon */
  safe_daemonHandle = osThreadNew(safe_daemon_task, NULL, &safe_daemon_attributes);

  /* creation of ins_task_rtos */
  ins_task_rtosHandle = osThreadNew(imu_task, NULL, &ins_task_rtos_attributes);

  /* creation of chassis */
  chassisHandle = osThreadNew(chassis_task, NULL, &chassis_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_test_task */
/**
* @brief Function implementing the test thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_test_task */
__weak void test_task(void *argument)
{
  /* USER CODE BEGIN test_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END test_task */
}

/* USER CODE BEGIN Header_remote_control_task */
/**
* @brief Function implementing the remote_control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_remote_control_task */
__weak void remote_control_task(void *argument)
{
  /* USER CODE BEGIN remote_control_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END remote_control_task */
}

/* USER CODE BEGIN Header_decision_center_task */
/**
* @brief Function implementing the decision_center thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_decision_center_task */
__weak void decision_center_task(void *argument)
{
  /* USER CODE BEGIN decision_center_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END decision_center_task */
}

/* USER CODE BEGIN Header_safe_daemon_task */
/**
* @brief Function implementing the safe_daemon thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_safe_daemon_task */
__weak void safe_daemon_task(void *argument)
{
  /* USER CODE BEGIN safe_daemon_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END safe_daemon_task */
}

/* USER CODE BEGIN Header_imu_task */
/**
* @brief Function implementing the ins_task_rtos thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_imu_task */
__weak void imu_task(void *argument)
{
  /* USER CODE BEGIN imu_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END imu_task */
}

/* USER CODE BEGIN Header_chassis_task */
/**
* @brief Function implementing the chassis thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_chassis_task */
__weak void chassis_task(void *argument)
{
  /* USER CODE BEGIN chassis_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END chassis_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
osThreadId AppRemote_Handle;;
osThreadId ChassisDrive_Handle;
osThreadId Vision_Handle;
osThreadId OffLine_Check_Handle;
/* USER CODE END Variables */
osThreadId MotorTextHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void AppRemote_Task(void const * argument);
void ChassisDrive_Task(void const * argument);
void Vision_Task(void const * argument);
void OffLine_Check_Task(void const * argument);
/* USER CODE END FunctionPrototypes */

void MotorTextTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

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
  /* definition and creation of MotorText */
  osThreadDef(MotorText, MotorTextTask, osPriorityNormal, 0, 128);
  MotorTextHandle = osThreadCreate(osThread(MotorText), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
/**************************************断线检测任务**************************************/
  osThreadDef(OffLine_Check, OffLine_Check_Task, osPriorityAboveNormal, 0, 256);
	OffLine_Check_Handle = osThreadCreate(osThread(OffLine_Check), NULL);
/**************************************数据处理任务**************************************/
  /*********App数据接收任务*********/
	osThreadDef(AppRemote, AppRemote_Task, osPriorityHigh, 0, 512);
	AppRemote_Handle = osThreadCreate(osThread(AppRemote), NULL);
	/*********视觉数据接收任务*********/
	osThreadDef(Vision, Vision_Task, osPriorityNormal, 0, 256);
	Vision_Handle = osThreadCreate(osThread(Vision), NULL);
/**************************************底盘控制任务**************************************/
  /**************驱动任务**************/
	osThreadDef(ChassisDrive, ChassisDrive_Task, osPriorityNormal, 0, 512);
	ChassisDrive_Handle = osThreadCreate(osThread(ChassisDrive), NULL);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_MotorTextTask */
/**
  * @brief  Function implementing the MotorText thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_MotorTextTask */
__weak void MotorTextTask(void const * argument)
{
  /* USER CODE BEGIN MotorTextTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END MotorTextTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

__weak void AppRemote_Task(void const * argument)
{
	for(;;)
	{
		osDelay(1);
	}
}

__weak void ChassisDrive_Task(void const * argument)
{
	for(;;)
	{
		osDelay(1);
	}
}

__weak void Vision_Task(void const * argument)
{
	for(;;)
	{
		osDelay(1);
	}
}

__weak void OffLine_Check_Task(void const * argument)
{
	for(;;)
	{
		osDelay(1);
	}
}
/* USER CODE END Application */

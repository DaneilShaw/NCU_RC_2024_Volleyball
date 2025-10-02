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
osThreadId CushionBall_Handle;
osThreadId AppRemote_Handle;
osThreadId ArmDrive_Handle;
osThreadId Vision_Handle;
osThreadId LightRemote_Handle;
//osThreadId Serve_Handle;
osThreadId HitBall_Handle;
/* USER CODE END Variables */
osThreadId MotorTextHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void CushionBall_Task(void const * argument);
void AppRemote_Task(void const * argument);
void ArmDrive_Task(void const * argument);
void Vision_Task(void const * argument);
void LightRemote_Task(void const * argument);
//void Serve_Task(void const * argument);
void HitBall_Task(void const * argument);
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
  osThreadDef(MotorText, MotorTextTask, osPriorityNormal, 0, 512);
  MotorTextHandle = osThreadCreate(osThread(MotorText), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
/**************************************数据处理任务**************************************/
  /*********App数据接收任务*********/
	osThreadDef(AppRemote, AppRemote_Task, osPriorityHigh, 0, 128);
	AppRemote_Handle = osThreadCreate(osThread(AppRemote), NULL);
	/*********视觉数据接收任务*********/
	osThreadDef(Vision, Vision_Task, osPriorityNormal, 0, 128);
	Vision_Handle = osThreadCreate(osThread(Vision), NULL);
	/*********光电传感器数据处理任务*********/
	osThreadDef(LightRemote, LightRemote_Task, osPriorityNormal, 0, 128);
	LightRemote_Handle = osThreadCreate(osThread(LightRemote), NULL);
/**************************************机械臂控制任务**************************************/
  /*********电机驱动任务*********/
	osThreadDef(ArmDrive, ArmDrive_Task, osPriorityNormal, 0, 128);
	ArmDrive_Handle = osThreadCreate(osThread(ArmDrive), NULL);
	/*********垫球任务*********/
	osThreadDef(CushionBall, CushionBall_Task, osPriorityNormal, 0, 128);
	CushionBall_Handle = osThreadCreate(osThread(CushionBall), NULL);
	/*********击球任务*********/
	osThreadDef(HitBall, HitBall_Task, osPriorityNormal, 0, 256);
	HitBall_Handle = osThreadCreate(osThread(HitBall), NULL);
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
__weak void CushionBall_Task(void const * argument)
{
	for(;;)
	{
		osDelay(1);
	}
}

__weak void AppRemote_Task(void const * argument)
{
	for(;;)
	{
		osDelay(1);
	}
}

__weak void ArmDrive_Task(void const * argument)
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

__weak void LightRemote_Task(void const * argument)
{
	for(;;)
	{
		osDelay(1);
	}
}

//__weak void Serve_Task(void const * argument)
//{
//	for(;;)
//	{
//		osDelay(1);
//	}
//}

__weak void HitBall_Task(void const * argument)
{
	for(;;)
	{
		osDelay(1);
	}
}
/* USER CODE END Application */

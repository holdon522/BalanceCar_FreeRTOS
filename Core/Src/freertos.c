/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stdio.h"
#include "oled.h"
#include "mpu6050.h"
#include "cartask.h"
#include "control.h"
#include "valuepack.h"
#include "usart.h"
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
extern unsigned char vp_rxbuff[VALUEPACK_BUFFER_SIZE];
extern unsigned char vp_txbuff[TXPACK_BYTE_SIZE + 3];
/* USER CODE END Variables */
osThreadId Task_SysHandle;
osThreadId Task_ControlHandle;
osThreadId Task_IMUHandle;
osThreadId Task_ConnectHandle;
osSemaphoreId myBinarySem_rxokHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartTask_5Hz(void const * argument);
void StartTask_100Hz(void const * argument);
void StartTask_200Hz(void const * argument);
void StartTask_Connect(void const * argument);

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

  /* Create the semaphores(s) */
  /* definition and creation of myBinarySem_rxok */
  osSemaphoreDef(myBinarySem_rxok);
  myBinarySem_rxokHandle = osSemaphoreCreate(osSemaphore(myBinarySem_rxok), 1);

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
  /* definition and creation of Task_Sys */
  osThreadDef(Task_Sys, StartTask_5Hz, osPriorityLow, 0, 128);
  Task_SysHandle = osThreadCreate(osThread(Task_Sys), NULL);

  /* definition and creation of Task_Control */
  osThreadDef(Task_Control, StartTask_100Hz, osPriorityNormal, 0, 128);
  Task_ControlHandle = osThreadCreate(osThread(Task_Control), NULL);

  /* definition and creation of Task_IMU */
  osThreadDef(Task_IMU, StartTask_200Hz, osPriorityBelowNormal, 0, 128);
  Task_IMUHandle = osThreadCreate(osThread(Task_IMU), NULL);

  /* definition and creation of Task_Connect */
  osThreadDef(Task_Connect, StartTask_Connect, osPriorityLow, 0, 128);
  Task_ConnectHandle = osThreadCreate(osThread(Task_Connect), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	vTaskSuspend(Task_ControlHandle);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartTask_5Hz */
/**
  * @brief  Function implementing the Task_5Hz thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask_5Hz */
void StartTask_5Hz(void const * argument)
{
  /* USER CODE BEGIN StartTask_5Hz */
	OLED_Init();
  /* Infinite loop */
  for(;;)
  {
		Car_Task_System();
    osDelay(200);
  }
  /* USER CODE END StartTask_5Hz */
}

/* USER CODE BEGIN Header_StartTask_100Hz */
/**
* @brief Function implementing the Task_100Hz thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_100Hz */
void StartTask_100Hz(void const * argument)
{
  /* USER CODE BEGIN StartTask_100Hz */
  /* Infinite loop */
  for(;;)
  {
		Car_Task_Motor();
    osDelay(20);
  }
  /* USER CODE END StartTask_100Hz */
}

/* USER CODE BEGIN Header_StartTask_200Hz */
/**
* @brief Function implementing the Task_200Hz thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_200Hz */
void StartTask_200Hz(void const * argument)
{
  /* USER CODE BEGIN StartTask_200Hz */
	uint8_t ret;
	ret = atk_ms6050_init();
	if (ret != 0)
	{
			printf("ATK-MS6050 init failed!\r\n");   
	}
	ret = atk_ms6050_dmp_init();
	if (ret != 0)
	{
			printf("ATK-MS6050 DMP init failed!\r\n");
	}
	vTaskResume(Task_ControlHandle);
  /* Infinite loop */
  for(;;)
  {
		Car_Task_IMU();
		osDelay(2);
  }
  /* USER CODE END StartTask_200Hz */
}

/* USER CODE BEGIN Header_StartTask_Connect */
/**
* @brief Function implementing the Task_Connect thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_Connect */
void StartTask_Connect(void const * argument)
{
  /* USER CODE BEGIN StartTask_Connect */

  /* Infinite loop */
  for(;;)
  {
//		osSemaphoreWait (myBinarySem_rxokHandle,osWaitForever);//等待二值信号量，只有等到了才会往下运行
		connect_read_data();
    osDelay(10);
  }
  /* USER CODE END StartTask_Connect */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


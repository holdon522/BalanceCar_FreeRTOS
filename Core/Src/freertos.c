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
/* Definitions for Task_Sys */
osThreadId_t Task_SysHandle;
const osThreadAttr_t Task_Sys_attributes = {
  .name = "Task_Sys",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Task_Control */
osThreadId_t Task_ControlHandle;
const osThreadAttr_t Task_Control_attributes = {
  .name = "Task_Control",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_IMU */
osThreadId_t Task_IMUHandle;
const osThreadAttr_t Task_IMU_attributes = {
  .name = "Task_IMU",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Task_Speed */
osThreadId_t Task_SpeedHandle;
const osThreadAttr_t Task_Speed_attributes = {
  .name = "Task_Speed",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartTask_5Hz(void *argument);
void StartTask_100Hz(void *argument);
void StartTask_200Hz(void *argument);
void StartTask04(void *argument);

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
  /* creation of Task_Sys */
  Task_SysHandle = osThreadNew(StartTask_5Hz, NULL, &Task_Sys_attributes);

  /* creation of Task_Control */
  Task_ControlHandle = osThreadNew(StartTask_100Hz, NULL, &Task_Control_attributes);

  /* creation of Task_IMU */
  Task_IMUHandle = osThreadNew(StartTask_200Hz, NULL, &Task_IMU_attributes);

  /* creation of Task_Speed */
  Task_SpeedHandle = osThreadNew(StartTask04, NULL, &Task_Speed_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	vTaskSuspend(Task_ControlHandle);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartTask_5Hz */
/**
  * @brief  Function implementing the Task_5Hz thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask_5Hz */
void StartTask_5Hz(void *argument)
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
void StartTask_100Hz(void *argument)
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
void StartTask_200Hz(void *argument)
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

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the Task_Speed thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
	RxPack rx;
	TxPack tx;
	uint8_t Senbuff[] = "\r\n**** Serial Output Message by DMA ***\r\n   UART DMA Test \r\n  ";
  /* Infinite loop */
  for(;;)
  {
//		HC05_Start();

//		if(readValuePack(&rx))
//		{
//			// 在此读取手机传来的数据			
//			// 这里是将接收的数据原样回传
//			tx.bools[0] = rx.bools[0];

//			tx.integers[0] = rx.integers[0];
//			tx.floats[0] = rx.floats[0];	
//			// 你也可以把 sendValuePack放在这，这样就只有当接收到手机传来的数据包后才回传数据	
//		}
//				readValuePack(&rx);
//		sendValuePack(&tx);
		HAL_UART_Transmit_DMA(&huart1, (uint8_t *)Senbuff, sizeof(Senbuff));
//		printf("rxbuf:%d\r\n,",vp_rxbuff[0]);
    osDelay(100);
  }
  /* USER CODE END StartTask04 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


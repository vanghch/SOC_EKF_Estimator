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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
osThreadId_t ledTaskHandle;
const osThreadAttr_t ledTask_attributes = {
  .name = "ledTask",
  .stack_size = 128,
  .priority = (osPriority_t) osPriorityLow,
};

osThreadId_t bq769xxTaskHandle;
const osThreadAttr_t bq769xxTask_attributes = {
  .name = "bq769xxTask",
  .stack_size = 256,
  .priority = (osPriority_t) osPriorityRealtime,
};

osThreadId_t socTaskHandle;
const osThreadAttr_t socTask_attributes = {
  .name = "socTask",
  .stack_size = 512,
  .priority = (osPriority_t) osPriorityHigh,
};

osThreadId_t publishstatusTaskHandle;
const osThreadAttr_t publishstatusTask_attributes = {
  .name = "publishstatusTask",
  .stack_size = 1024,
  .priority = (osPriority_t) osPriorityNormal,
};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for startTask */
osThreadId_t startTaskHandle;
const osThreadAttr_t startTask_attributes = {
  .name = "startTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for battery_status_bq769xx_to_soc */
osMessageQueueId_t battery_status_bq769xx_to_socHandle;
const osMessageQueueAttr_t battery_status_bq769xx_to_soc_attributes = {
  .name = "battery_status_bq769xx_to_soc"
};
/* Definitions for soc_status_soc_to_publish */
osMessageQueueId_t soc_status_soc_to_publishHandle;
const osMessageQueueAttr_t soc_status_soc_to_publish_attributes = {
  .name = "soc_status_soc_to_publish"
};
/* Definitions for battery_status_bq769xx_to_publish */
osMessageQueueId_t battery_status_bq769xx_to_publishHandle;
const osMessageQueueAttr_t battery_status_bq769xx_to_publish_attributes = {
  .name = "battery_status_bq769xx_to_publish"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void LedTaskFunc(void *argument);
void Bq769xxTaskFunc(void *argument);
void SocEstimatorFunc(void *argument);
void PublishStatusFunc(void *argument);

/* USER CODE END FunctionPrototypes */

void StartTaskFunc(void *argument);

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

  /* Create the queue(s) */
  /* creation of battery_status_bq769xx_to_soc */
  battery_status_bq769xx_to_socHandle = osMessageQueueNew (1, sizeof(BatteryStatusStructure), &battery_status_bq769xx_to_soc_attributes);

  /* creation of soc_status_soc_to_publish */
  soc_status_soc_to_publishHandle = osMessageQueueNew (1, sizeof(SocStatusStructure), &soc_status_soc_to_publish_attributes);

  /* creation of battery_status_bq769xx_to_publish */
  battery_status_bq769xx_to_publishHandle = osMessageQueueNew (1, sizeof(BatteryStatusStructure), &battery_status_bq769xx_to_publish_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of startTask */
  startTaskHandle = osThreadNew(StartTaskFunc, NULL, &startTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* creation of rtos_threads */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartTaskFunc */
/**
  * @brief  Function implementing the startTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTaskFunc */
void StartTaskFunc(void *argument)
{
  /* USER CODE BEGIN StartTaskFunc */
  ledTaskHandle = osThreadNew(LedTaskFunc, NULL, &ledTask_attributes);
  if (NULL == ledTaskHandle) {
    RTOS_ERROR("%s is NULL", ledTask_attributes.name);
  }
  bq769xxTaskHandle = osThreadNew(Bq769xxTaskFunc, NULL, &bq769xxTask_attributes);
  if (NULL == bq769xxTaskHandle) {
    RTOS_ERROR("%s is NULL", bq769xxTask_attributes.name);
  }
  socTaskHandle = osThreadNew(SocEstimatorFunc, NULL, &socTask_attributes);
  if (NULL == socTaskHandle) {
    RTOS_ERROR("%s is NULL", socTask_attributes.name);
  }
  publishstatusTaskHandle = osThreadNew(PublishStatusFunc, NULL, &publishstatusTask_attributes);
  if (NULL == publishstatusTaskHandle) {
    RTOS_ERROR("%s is NULL", publishstatusTask_attributes.name);
  }

  vTaskDelete(NULL);

  osThreadExit();
    
  /* USER CODE END StartTaskFunc */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void LedTaskFunc(void *argument)
{
  for(;;)
  {
    led_flash();
  }
}

void Bq769xxTaskFunc(void *argument)
{
  for(;;)
  {
    osDelay(10);
    system_discharge_check();
    BQ769xx_get_data();
    battery_status_publish();
  }
}

void SocEstimatorFunc(void *argument)
{
  for(;;)
  {
    osDelay(50);
    soc_update();
  }
}

void PublishStatusFunc(void *argument)
{
  osDelay(3000);
  for(;;)
  {
    osDelay(50);
    publish_update();
  }
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

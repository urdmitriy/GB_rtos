/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#define BIT_0 (1<<0)
#define BIT_1 (1<<1)
#define BIT_2 (1<<2)
#define BIT_3 (1<<3)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

osThreadId defaultTaskHandle;
osThreadId TaskButtonHandle;
osThreadId TaskLed1Handle;
osThreadId TaskLed2Handle;
osThreadId TaskLed3Handle;
osThreadId TaskLed4Handle;
/* USER CODE BEGIN PV */
EventGroupHandle_t eventGroup;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartDefaultTask(void const * argument);
void StartTaskButton(void const * argument);
void StartTaskLed1(void const * argument);
void StartTaskLed2(void const * argument);
void StartTaskLed3(void const * argument);
void StartTaskLed4(void const * argument);

/* USER CODE BEGIN PFP */
void led_flash(GPIO_TypeDef* port, uint16_t pin);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
    eventGroup = xEventGroupCreate();

  /* USER CODE END 2 */

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

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of TaskButton */
  osThreadDef(TaskButton, StartTaskButton, osPriorityNormal, 0, 128);
  TaskButtonHandle = osThreadCreate(osThread(TaskButton), NULL);

  /* definition and creation of TaskLed1 */
  osThreadDef(TaskLed1, StartTaskLed1, osPriorityIdle, 0, 128);
  TaskLed1Handle = osThreadCreate(osThread(TaskLed1), NULL);

  /* definition and creation of TaskLed2 */
  osThreadDef(TaskLed2, StartTaskLed2, osPriorityIdle, 0, 128);
  TaskLed2Handle = osThreadCreate(osThread(TaskLed2), NULL);

  /* definition and creation of TaskLed3 */
  osThreadDef(TaskLed3, StartTaskLed3, osPriorityIdle, 0, 128);
  TaskLed3Handle = osThreadCreate(osThread(TaskLed3), NULL);

  /* definition and creation of TaskLed4 */
  osThreadDef(TaskLed4, StartTaskLed4, osPriorityIdle, 0, 128);
  TaskLed4Handle = osThreadCreate(osThread(TaskLed4), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LED2_Pin|LED1_Pin|LD3_Pin
                          |LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED4_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LED2_Pin LED1_Pin LD3_Pin
                           LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LED2_Pin|LED1_Pin|LD3_Pin
                          |LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON3_Pin BUTTON4_Pin BUTTON2_Pin BUTTON1_Pin */
  GPIO_InitStruct.Pin = BUTTON3_Pin|BUTTON4_Pin|BUTTON2_Pin|BUTTON1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LED4_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED4_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void led_flash(GPIO_TypeDef* port, uint16_t pin){
    HAL_GPIO_WritePin(port, pin, SET);
    osDelay(100);
    HAL_GPIO_WritePin(port, pin, RESET);
    osDelay(1);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {

      osDelay(100);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTaskButton */
/**
* @brief Function implementing the TaskButton thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskButton */
void StartTaskButton(void const * argument)
{
  /* USER CODE BEGIN StartTaskButton */
  /* Infinite loop */
  for(;;)
  {
      EventBits_t eventBits;

      if (HAL_GPIO_ReadPin(BUTTON1_GPIO_Port, BUTTON1_Pin) == RESET) {
          xEventGroupSetBits(eventGroup, BIT_0);
      }

      if (HAL_GPIO_ReadPin(BUTTON2_GPIO_Port, BUTTON2_Pin) == RESET) {
          xEventGroupSetBits(eventGroup, BIT_1);
      }

      if (HAL_GPIO_ReadPin(BUTTON3_GPIO_Port, BUTTON3_Pin) == RESET) {
          xEventGroupSetBits(eventGroup, BIT_2);
      }

      if (HAL_GPIO_ReadPin(BUTTON4_GPIO_Port, BUTTON4_Pin) == RESET) {
          xEventGroupSetBits(eventGroup, BIT_3);
      }

      osDelay(2);
  }
  /* USER CODE END StartTaskButton */
}

/* USER CODE BEGIN Header_StartTaskLed1 */
/**
* @brief Function implementing the TaskLed1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskLed1 */
void StartTaskLed1(void const * argument)
{
  /* USER CODE BEGIN StartTaskLed1 */
  /* Infinite loop */
  for(;;)
  {
      xEventGroupWaitBits(eventGroup, BIT_0|BIT_3, pdTRUE, pdTRUE, portMAX_DELAY);
      led_flash(LED1_GPIO_Port,LED1_Pin);
  }
  /* USER CODE END StartTaskLed1 */
}

/* USER CODE BEGIN Header_StartTaskLed2 */
/**
* @brief Function implementing the TaskLed2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskLed2 */
void StartTaskLed2(void const * argument)
{
  /* USER CODE BEGIN StartTaskLed2 */
  /* Infinite loop */
  for(;;)
  {
      xEventGroupWaitBits(eventGroup, BIT_0, pdTRUE, pdTRUE, portMAX_DELAY);
      led_flash(LED2_GPIO_Port, LED2_Pin);
  }
  /* USER CODE END StartTaskLed2 */
}

/* USER CODE BEGIN Header_StartTaskLed3 */
/**
* @brief Function implementing the TaskLed3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskLed3 */
void StartTaskLed3(void const * argument)
{
  /* USER CODE BEGIN StartTaskLed3 */
  /* Infinite loop */
  for(;;)
  {
      xEventGroupWaitBits(eventGroup, BIT_2|BIT_3, pdTRUE, pdTRUE, portMAX_DELAY);
      led_flash(LED3_GPIO_Port, LED3_Pin);
  }
  /* USER CODE END StartTaskLed3 */
}

/* USER CODE BEGIN Header_StartTaskLed4 */
/**
* @brief Function implementing the TaskLed3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskLed4 */
void StartTaskLed4(void const * argument)
{
  /* USER CODE BEGIN StartTaskLed4 */
  /* Infinite loop */
  for(;;)
  {
      xEventGroupWaitBits(eventGroup, BIT_0|BIT_1|BIT_2|BIT_3, pdTRUE, pdFALSE, portMAX_DELAY);
      HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, SET);
      osDelay(100);
      HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, RESET);
      osDelay(1);
  }
  /* USER CODE END StartTaskLed4 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

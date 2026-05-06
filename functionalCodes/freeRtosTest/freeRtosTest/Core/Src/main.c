/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum State {
	STATE_OFF,
	STATE_LOW,
	STATE_BOTH,
} State;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define LOW_FREQ_PERIOD_MS 50
#define HIGH_FREQ_PERIOD_MS 45
#define LED_LOW_FREQ_EVT_BIT 0x01
#define LED_HIGH_FREQ_EVT_BIT 0x02
#define BUTTON_EVT_BIT 0x04
#define BUF_MAX_SIZE 0x10

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define CONST_MS_TO_TICKS(ms) ms

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ledLowFreqTask */
osThreadId_t ledLowFreqTaskHandle;
const osThreadAttr_t ledLowFreqTask_attributes = {
  .name = "ledLowFreqTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ledHighFreqTask */
osThreadId_t ledHighFreqTaskHandle;
const osThreadAttr_t ledHighFreqTask_attributes = {
  .name = "ledHighFreqTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for lowFreqTimer */
osTimerId_t lowFreqTimerHandle;
const osTimerAttr_t lowFreqTimer_attributes = {
  .name = "lowFreqTimer"
};
/* Definitions for highFreqTimer */
osTimerId_t highFreqTimerHandle;
const osTimerAttr_t highFreqTimer_attributes = {
  .name = "highFreqTimer"
};
/* Definitions for ledMutex */
osMutexId_t ledMutexHandle;
const osMutexAttr_t ledMutex_attributes = {
  .name = "ledMutex"
};
/* Definitions for mainEvent */
osEventFlagsId_t mainEventHandle;
const osEventFlagsAttr_t mainEvent_attributes = {
  .name = "mainEvent"
};
/* USER CODE BEGIN PV */

uint8_t ledState = 0u;
State state = STATE_OFF;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);
void StartLedLowFreqTask(void *argument);
void StartLedHighFreqTask(void *argument);
void LowFreqCallback(void *argument);
void HighFreqCallback(void *argument);

/* USER CODE BEGIN PFP */

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of ledMutex */
  ledMutexHandle = osMutexNew(&ledMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of lowFreqTimer */
  lowFreqTimerHandle = osTimerNew(LowFreqCallback, osTimerPeriodic, NULL, &lowFreqTimer_attributes);

  /* creation of highFreqTimer */
  highFreqTimerHandle = osTimerNew(HighFreqCallback, osTimerPeriodic, NULL, &highFreqTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */

  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ledLowFreqTask */
  ledLowFreqTaskHandle = osThreadNew(StartLedLowFreqTask, NULL, &ledLowFreqTask_attributes);

  /* creation of ledHighFreqTask */
  ledHighFreqTaskHandle = osThreadNew(StartLedHighFreqTask, NULL, &ledHighFreqTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */

  /* USER CODE END RTOS_THREADS */

  /* creation of mainEvent */
  mainEventHandle = osEventFlagsNew(&mainEvent_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
	printf("interrupt\r\n");
	osEventFlagsSet(mainEventHandle, BUTTON_EVT_BIT);
}

void _write(int file, char *ptr, int len)
{
	uint8_t buf[BUF_MAX_SIZE];

	while (len > BUF_MAX_SIZE) {
		memcpy(buf, ptr, BUF_MAX_SIZE);
		HAL_UART_Transmit(&huart2, buf, BUF_MAX_SIZE, 1000);

		len -= BUF_MAX_SIZE;
		ptr += BUF_MAX_SIZE;
	}

	memcpy(buf, ptr, len);
	HAL_UART_Transmit(&huart2, buf, len, 1000);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */

  printf("defaultTask started\r\n");
  for(;;)
  {
	  printf("defaultTask loop\r\n");

	  switch (state) {
	  case STATE_OFF:
		  osTimerStop(lowFreqTimerHandle);
		  osTimerStop(highFreqTimerHandle);
		  state = STATE_LOW;
		  break;
	  case STATE_LOW:
		  osTimerStart(lowFreqTimerHandle, CONST_MS_TO_TICKS(LOW_FREQ_PERIOD_MS));
		  state = STATE_BOTH;
		  break;
	  case STATE_BOTH:
		  osTimerStart(highFreqTimerHandle, CONST_MS_TO_TICKS(HIGH_FREQ_PERIOD_MS));
		  state = STATE_OFF;
		  break;
	  default:
		  state = STATE_OFF;
		  break;
	  }

	  osEventFlagsWait(mainEventHandle, BUTTON_EVT_BIT, osFlagsWaitAny, osWaitForever);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartLedLowFreqTask */
/**
* @brief Function implementing the ledLowFreqTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLedLowFreqTask */
void StartLedLowFreqTask(void *argument)
{
  /* USER CODE BEGIN StartLedLowFreqTask */
  /* Infinite loop */
  for(;;)
  {
	  osEventFlagsWait(mainEventHandle, LED_LOW_FREQ_EVT_BIT, osFlagsWaitAny, osWaitForever);

	  osMutexAcquire(ledMutexHandle, osWaitForever);

	  printf("LowFreqTask\r\n");
	  ledState = !ledState;
	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, ledState);

	  osMutexRelease(ledMutexHandle);
  }
  /* USER CODE END StartLedLowFreqTask */
}

/* USER CODE BEGIN Header_StartLedHighFreqTask */
/**
* @brief Function implementing the StartLedHighFre thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLedHighFreqTask */
void StartLedHighFreqTask(void *argument)
{
  /* USER CODE BEGIN StartLedHighFreqTask */
  /* Infinite loop */
  for(;;)
  {
	  osEventFlagsWait(mainEventHandle, LED_HIGH_FREQ_EVT_BIT, osFlagsWaitAny, osWaitForever);

	  osMutexAcquire(ledMutexHandle, osWaitForever);

	  printf("HighFreqTask\r\n");
	  ledState = !ledState;
	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, ledState);

	  osMutexRelease(ledMutexHandle);
  }
  /* USER CODE END StartLedHighFreqTask */
}

/* LowFreqCallback function */
void LowFreqCallback(void *argument)
{
  /* USER CODE BEGIN LowFreqCallback */
	osEventFlagsSet(mainEventHandle, LED_LOW_FREQ_EVT_BIT);
  /* USER CODE END LowFreqCallback */
}

/* HighFreqCallback function */
void HighFreqCallback(void *argument)
{
  /* USER CODE BEGIN HighFreqCallback */
	osEventFlagsSet(mainEventHandle, LED_HIGH_FREQ_EVT_BIT);
  /* USER CODE END HighFreqCallback */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
#ifdef USE_FULL_ASSERT
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

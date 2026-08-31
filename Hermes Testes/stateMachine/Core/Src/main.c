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
#include <stdarg.h>
#include <stdbool.h>
#include "customTypes.h"
#include "BME280.h"
#include "DS3231.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USART1_RX_FLAG       (1U << 0)
#define USART1_ERROR_FLAG    (1U << 1)
#define TASK_START_FLAG      (1U << 0)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* Definitions for mainTask */
osThreadId_t mainTaskHandle;
const osThreadAttr_t mainTask_attributes = {
  .name = "mainTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for housekeepTask */
osThreadId_t housekeepTaskHandle;
const osThreadAttr_t housekeepTask_attributes = {
  .name = "housekeepTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for transmitTask */
osThreadId_t transmitTaskHandle;
const osThreadAttr_t transmitTask_attributes = {
  .name = "transmitTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for receiveTask */
osThreadId_t receiveTaskHandle;
const osThreadAttr_t receiveTask_attributes = {
  .name = "receiveTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for deployTask */
osThreadId_t deployTaskHandle;
const osThreadAttr_t deployTask_attributes = {
  .name = "deployTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for detumblingTask */
osThreadId_t detumblingTaskHandle;
const osThreadAttr_t detumblingTask_attributes = {
  .name = "detumblingTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for missionTask */
osThreadId_t missionTaskHandle;
const osThreadAttr_t missionTask_attributes = {
  .name = "missionTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for eventQueue */
osMessageQueueId_t eventQueueHandle;
const osMessageQueueAttr_t eventQueue_attributes = {
  .name = "eventQueue"
};
/* Definitions for transmitQueue */
osMessageQueueId_t transmitQueueHandle;
const osMessageQueueAttr_t transmitQueue_attributes = {
  .name = "transmitQueue"
};
/* Definitions for HkTickTimer */
osTimerId_t HkTickTimerHandle;
const osTimerAttr_t HkTickTimer_attributes = {
  .name = "HkTickTimer"
};
/* Definitions for uartMutex */
osMutexId_t uartMutexHandle;
const osMutexAttr_t uartMutex_attributes = {
  .name = "uartMutex"
};
/* USER CODE BEGIN PV */
static uint8_t usart1RxBuffer;
static volatile uint8_t pendingTelecommand;
static volatile uint32_t usart1LastError;
static volatile uint32_t usart1ErrorCount;
static volatile bool usart1RestartRequired;
static volatile bool antennaDeployed;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
void StartMainTask(void *argument);
void StartHousekeepTask(void *argument);
void StartTransmitTask(void *argument);
void StartReceiveTask(void *argument);
void StartDeployTask(void *argument);
void StartDetumblingTask(void *argument);
void StartMissionTask(void *argument);
void vHkTickCallback(void *argument);

/* USER CODE BEGIN PFP */
static void debugPrintf(const char *format, ...);
static void handleTelecommand(uint8_t tc, SatMode_t *mode,
		bool *deployInProgress, volatile bool *deployCompleted);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 10);
	return ch;
}

static void debugPrintf(const char *format, ...)
{
	va_list args;

	if (osMutexAcquire(uartMutexHandle, osWaitForever) != osOK) {
		return;
	}

	va_start(args, format);
	vprintf(format, args);
	va_end(args);

	osMutexRelease(uartMutexHandle);
}

static void handleTelecommand(uint8_t tc, SatMode_t *mode,
		bool *deployInProgress, volatile bool *deployCompleted)
{
	switch ((Telecommand_t)tc) {
	case TC_START_MISSION:
		if ((*mode == SAT_MODE_IDLE) && !(*deployInProgress)) {
			*mode = SAT_MODE_MISSION;
			osThreadFlagsSet(missionTaskHandle, TASK_START_FLAG);
			debugPrintf("[%08lu] rx accepted: mission\r\n",
					osKernelGetTickCount());
		} else {
			debugPrintf("[%08lu] rx rejected: mission, mode=%u, deploy=%u\r\n",
					osKernelGetTickCount(), (unsigned int)*mode,
					(unsigned int)*deployInProgress);
		}
		break;

	case TC_DEPLOY:
		if ((*mode == SAT_MODE_IDLE) && !(*deployInProgress)
				&& !(*deployCompleted)) {
			*deployInProgress = true;
			osThreadFlagsSet(deployTaskHandle, TASK_START_FLAG);
			debugPrintf("[%08lu] rx accepted: deploy\r\n",
					osKernelGetTickCount());
		} else {
			debugPrintf("[%08lu] rx rejected: deploy, mode=%u, active=%u, done=%u\r\n",
					osKernelGetTickCount(), (unsigned int)*mode,
					(unsigned int)*deployInProgress,
					(unsigned int)*deployCompleted);
		}
		break;

	case TC_START_DETUMBLING:
		if ((*mode == SAT_MODE_IDLE) && !(*deployInProgress)) {
			*mode = SAT_MODE_DETUMBLING;
			osThreadFlagsSet(detumblingTaskHandle, TASK_START_FLAG);
			debugPrintf("[%08lu] rx accepted: detumbling\r\n",
					osKernelGetTickCount());
		} else {
			debugPrintf("[%08lu] rx rejected: detumbling, mode=%u, deploy=%u\r\n",
					osKernelGetTickCount(), (unsigned int)*mode,
					(unsigned int)*deployInProgress);
		}
		break;

	default:
		debugPrintf("[%08lu] rx rejected: unknown command 0x%02X\r\n",
				osKernelGetTickCount(), tc);
		break;
	}
}
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of uartMutex */
  uartMutexHandle = osMutexNew(&uartMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of HkTickTimer */
  HkTickTimerHandle = osTimerNew(vHkTickCallback, osTimerPeriodic, NULL, &HkTickTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of eventQueue */
  eventQueueHandle = osMessageQueueNew (16, sizeof(Event_t), &eventQueue_attributes);

  /* creation of transmitQueue */
  transmitQueueHandle = osMessageQueueNew (4, sizeof(Telemetry_t*), &transmitQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of mainTask */
  mainTaskHandle = osThreadNew(StartMainTask, NULL, &mainTask_attributes);

  /* creation of housekeepTask */
  housekeepTaskHandle = osThreadNew(StartHousekeepTask, NULL, &housekeepTask_attributes);

  /* creation of transmitTask */
  transmitTaskHandle = osThreadNew(StartTransmitTask, NULL, &transmitTask_attributes);

  /* creation of receiveTask */
  receiveTaskHandle = osThreadNew(StartReceiveTask, NULL, &receiveTask_attributes);

  /* creation of deployTask */
  deployTaskHandle = osThreadNew(StartDeployTask, NULL, &deployTask_attributes);

  /* creation of detumblingTask */
  detumblingTaskHandle = osThreadNew(StartDetumblingTask, NULL, &detumblingTask_attributes);

  /* creation of missionTask */
  missionTaskHandle = osThreadNew(StartMissionTask, NULL, &missionTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00100D14;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void initializeEverything(BME280_HandleTypeDef *BMEBat) {
	Set_Time(0, 0, 0, 0, 0, 0, 0);
	BME280_initDefault(BMEBat, hi2c1);
	BME280_setOversampling(BMEBat, BME280_OS_16x, PSSR); // talvez devesse ser ALL aqui ao invés de PSSR
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uint32_t flags = USART1_RX_FLAG;

        pendingTelecommand = usart1RxBuffer;

        if (HAL_UART_Receive_IT(&huart1, &usart1RxBuffer, 1U) != HAL_OK)
        {
            usart1LastError = HAL_UART_GetError(&huart1);
            usart1ErrorCount++;
            usart1RestartRequired = true;
            flags |= USART1_ERROR_FLAG;
        }

        (void)osThreadFlagsSet(receiveTaskHandle, flags);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        usart1LastError = HAL_UART_GetError(huart);
        usart1ErrorCount++;

        if (huart->RxState == HAL_UART_STATE_READY)
        {
            usart1RestartRequired =
                    (HAL_UART_Receive_IT(&huart1, &usart1RxBuffer, 1U)
                            != HAL_OK);
        }
        else
        {
            usart1RestartRequired = false;
        }

        (void)osThreadFlagsSet(receiveTaskHandle, USART1_ERROR_FLAG);
    }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartMainTask */
/**
  * @brief  Function implementing the mainTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMainTask */
void StartMainTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	Event_t evt;
	SatMode_t mode = SAT_MODE_IDLE;
	bool deployInProgress = false;

	osTimerStart(HkTickTimerHandle, pdMS_TO_TICKS(60000));
	debugPrintf("[%08lu] main ready, mode=%u\r\n",
			osKernelGetTickCount(), (unsigned int)mode);

	/* Infinite loop */
	for(;;)
	{
		if (osMessageQueueGet(eventQueueHandle, &evt, NULL,
				osWaitForever) != osOK) {
			continue;
		}

		switch (evt.type) {
		case EVT_HK_DONE: {
			Telemetry_t *packet = evt.data.telemetry;

			if (osMessageQueuePut(transmitQueueHandle, &packet, 0U,
					osWaitForever) != osOK) {
				debugPrintf("[%08lu] tx queue error\r\n",
						osKernelGetTickCount());
			}
			break;
		}

		case EVT_RX_DONE:
			handleTelecommand(evt.data.telecommand, &mode,
					&deployInProgress, &antennaDeployed);
			break;

		case EVT_TX_DONE:
			debugPrintf("[%08lu] tx done\r\n", evt.timestamp);
			break;

		case EVT_MISSION_DONE:
			if (mode == SAT_MODE_MISSION) {
				mode = SAT_MODE_IDLE;
				debugPrintf("[%08lu] mission done, mode=%u\r\n",
						evt.timestamp, (unsigned int)mode);
			}
			break;

		case EVT_DETUMBLING_DONE:
			if (mode == SAT_MODE_DETUMBLING) {
				mode = SAT_MODE_IDLE;
				debugPrintf("[%08lu] detumbling done, mode=%u\r\n",
						evt.timestamp, (unsigned int)mode);
			}
			break;

		case EVT_DEPLOY_DONE:
			if (deployInProgress) {
				deployInProgress = false;
				antennaDeployed = true;
				debugPrintf("[%08lu] deploy done\r\n", evt.timestamp);
			}
			break;

		default:
			debugPrintf("[%08lu] unknown event=%u\r\n",
					evt.timestamp, (unsigned int)evt.type);
			break;
		}
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartHousekeepTask */
/**
* @brief Function implementing the housekeepTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartHousekeepTask */
void StartHousekeepTask(void *argument)
{
  /* USER CODE BEGIN StartHousekeepTask */
	static Telemetry_t telemetryPacket;
	static BME280_HandleTypeDef BMEBat;
	initializeEverything(&BMEBat);
  /* Infinite loop */
  for(;;)
  {
	  osThreadFlagsWait(TASK_START_FLAG, osFlagsWaitAny, osWaitForever);
	  // ler os sensores aqui...
	  BME280_readSingleShot(&BMEBat);

	  // criar a struct para transmitir
	  telemetryPacket.temperatura_bateria = telemetryPacket.temperatura_estrutura = BMEBat.READ.TEMP;
	  telemetryPacket.umidade = BMEBat.READ.HMDT;
	  telemetryPacket.pressao = BMEBat.READ.PSSR;
	  Get_Time(&telemetryPacket.segundo, &telemetryPacket.minuto, &telemetryPacket.hora, &telemetryPacket.dia, NULL, &telemetryPacket.mes, &telemetryPacket.ano);
	  telemetryPacket.deploy_antena = antennaDeployed ? 1U : 0U;

	  osMutexAcquire(uartMutexHandle, osWaitForever);
	  uint8_t marker = 'H';
	  HAL_UART_Transmit(&huart2, &marker, 1, 100);
	  osMutexRelease(uartMutexHandle);

	  Event_t evt = {
		  .type = EVT_HK_DONE,
		  .timestamp = osKernelGetTickCount(),
		  .data.telemetry = &telemetryPacket
	  };
	  osMessageQueuePut(eventQueueHandle, &evt, 0U, osWaitForever);
  }
  /* USER CODE END StartHousekeepTask */
}

/* USER CODE BEGIN Header_StartTransmitTask */
/**
* @brief Function implementing the transmitTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTransmitTask */
void StartTransmitTask(void *argument)
{
  /* USER CODE BEGIN StartTransmitTask */
	Telemetry_t* telemetryPacket;
  /* Infinite loop */
  for(;;)
  {
	  osMessageQueueGet(transmitQueueHandle, &telemetryPacket, NULL, osWaitForever);
	  // send telemetry to the earth
	  osMutexAcquire(uartMutexHandle, osWaitForever);
	  uint8_t marker = 'T';
	  HAL_UART_Transmit(&huart2, &marker, 1, 100);
	  HAL_UART_Transmit(&huart2, (uint8_t*) telemetryPacket, sizeof(Telemetry_t), 1000);
	  osMutexRelease(uartMutexHandle);

	  Event_t evt = {
		  .type = EVT_TX_DONE,
		  .timestamp = osKernelGetTickCount()
	  };
	  osMessageQueuePut(eventQueueHandle, &evt, 0U, osWaitForever);
  }
  /* USER CODE END StartTransmitTask */
}

/* USER CODE BEGIN Header_StartReceiveTask */
/**
* @brief Function implementing the receiveTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReceiveTask */
void StartReceiveTask(void *argument)
{
  /* USER CODE BEGIN StartReceiveTask */
    uint32_t flags;

    if (HAL_UART_Receive_IT(&huart1, &usart1RxBuffer, 1U) != HAL_OK)
    {
        Error_Handler();
    }

    for (;;)
    {
        flags = osThreadFlagsWait(USART1_RX_FLAG | USART1_ERROR_FLAG,
                osFlagsWaitAny, osWaitForever);

        if ((flags & osFlagsError) != 0U)
        {
            continue;
        }

        if ((flags & USART1_ERROR_FLAG) != 0U)
        {
            if (usart1RestartRequired
                    && (huart1.RxState == HAL_UART_STATE_READY))
            {
                usart1RestartRequired =
                        (HAL_UART_Receive_IT(&huart1, &usart1RxBuffer, 1U)
                                != HAL_OK);
            }

            debugPrintf("[%08lu] usart1 error=0x%08lX count=%lu restart=%u\r\n",
                    osKernelGetTickCount(), usart1LastError,
                    usart1ErrorCount, (unsigned int)usart1RestartRequired);
        }

        if ((flags & USART1_RX_FLAG) != 0U)
        {
            Event_t evt = {
                .type = EVT_RX_DONE,
                .timestamp = osKernelGetTickCount(),
                .data.telecommand = pendingTelecommand
            };

            osMessageQueuePut(eventQueueHandle, &evt, 0U, osWaitForever);
        }
    }

  /* USER CODE END StartReceiveTask */
}

/* USER CODE BEGIN Header_StartDeployTask */
/**
* @brief Function implementing the deployTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDeployTask */
void StartDeployTask(void *argument)
{
  /* USER CODE BEGIN StartDeployTask */
  /* Infinite loop */
  for(;;)
  {
	  osThreadFlagsWait(TASK_START_FLAG, osFlagsWaitAny, osWaitForever);
	  debugPrintf("[%08lu] deploy task ran\r\n", osKernelGetTickCount());
	  Event_t evt = {
		  .type = EVT_DEPLOY_DONE,
		  .timestamp = osKernelGetTickCount()
	  };
	  osMessageQueuePut(eventQueueHandle, &evt, 0U, osWaitForever);
  }
  /* USER CODE END StartDeployTask */
}

/* USER CODE BEGIN Header_StartDetumblingTask */
/**
* @brief Function implementing the detumblingTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDetumblingTask */
void StartDetumblingTask(void *argument)
{
  /* USER CODE BEGIN StartDetumblingTask */
  /* Infinite loop */
  for(;;)
  {
	  osThreadFlagsWait(TASK_START_FLAG, osFlagsWaitAny, osWaitForever);
	  debugPrintf("[%08lu] detumbling task ran\r\n", osKernelGetTickCount());
	  Event_t evt = {
		  .type = EVT_DETUMBLING_DONE,
		  .timestamp = osKernelGetTickCount()
	  };
	  osMessageQueuePut(eventQueueHandle, &evt, 0U, osWaitForever);
  }
  /* USER CODE END StartDetumblingTask */
}

/* USER CODE BEGIN Header_StartMissionTask */
/**
* @brief Function implementing the missionTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMissionTask */
void StartMissionTask(void *argument)
{
  /* USER CODE BEGIN StartMissionTask */
  /* Infinite loop */
  for(;;)
  {
	  osThreadFlagsWait(TASK_START_FLAG, osFlagsWaitAny, osWaitForever);
	  debugPrintf("[%08lu] mission task ran\r\n", osKernelGetTickCount());
	  Event_t evt = {
		  .type = EVT_MISSION_DONE,
		  .timestamp = osKernelGetTickCount()
	  };
	  osMessageQueuePut(eventQueueHandle, &evt, 0U, osWaitForever);
  }
  /* USER CODE END StartMissionTask */
}

/* vHkTickCallback function */
void vHkTickCallback(void *argument)
{
  /* USER CODE BEGIN vHkTickCallback */
	osThreadFlagsSet(housekeepTaskHandle, TASK_START_FLAG);
  /* USER CODE END vHkTickCallback */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
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

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
#include <string.h>
#include "customTypes.h"
#include "BME280.h"
#include "DS3231.h"
#include "hermes_adsb.h"
#if MONITOR_USB
#include "monitor_usb.h"   /* bancada na PCB: functionalCodes/bancadaStateMachine/usb_monitor */
#endif
#include "payload_downlink.h"
#include "adcs_link.h"
#include "eps.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USART1_RX_FLAG       (1U << 0)
#define USART1_ERROR_FLAG    (1U << 1)
#define USART1_ATIVIDADE_FLAG (1U << 2)   /* byte na USART1: reavalia o candidato a telecomando */
/* Telecomando do TT&C = 1 byte isolado na USART1. O TT&C (ESP32) usa a UART0 (RX0/TX0) para o OBC, e a ROM
 * do ESP32 imprime o log de boot (115200) nela a cada reset: a 9600 isso chega como uma rajada de bytes
 * aleatórios, alguns na faixa 0x00-0x0F (0x02 = Deploy). Só vira telecomando o byte com pelo menos
 * TC_SILENCIO_MS de silêncio (sem bytes nem erros) antes e depois. */
#define TC_SILENCIO_MS       30U
/* TTC_BUSY (PC0, J2-10 do barramento): o TT&C põe em alto enquanto o rádio está ocupado ou há um quadro
 * esperando o rádio. Antes de cada quadro, o OBC espera o busy baixar, por no máximo TTC_BUSY_ESPERA_MS;
 * depois disso manda assim mesmo (o TT&C guarda até 1 KB na UART). */
#define TTC_BUSY_PORT        GPIOC
#define TTC_BUSY_PIN         GPIO_PIN_0
#define TTC_BUSY_ESPERA_MS   2000U
/* Deploy das duas antenas (conector J3). CTRL_ANT_n em alto liga o MOSFET que queima o fio da antena n.
 * O fio pressiona um botão ligado ao GND (MT_n): baixo = fio inteiro; alto (pull-up interno) = fio rompeu.
 * Uma antena por vez, até o botão abrir ou DEPLOY_QUEIMA_MAX_MS. */
#define DEPLOY_QUEIMA_MAX_MS 15000U
#define DEPLOY_POLL_MS       10U
#define TASK_START_FLAG      (1U << 0)
#define TASK_STOP_FLAG       (1U << 1)

#define ENABLE_DEBUG_UART2   (!ADCS_NA_USART2)   /* na PCB a USART2 é do ADCS */
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
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for transmitTask */
osThreadId_t transmitTaskHandle;
const osThreadAttr_t transmitTask_attributes = {
  .name = "transmitTask",
#if MONITOR_USB || ENABLE_DEBUG_UART2
  /* com debug, o debugTelemetria() formata texto (vsnprintf) nesta pilha: 512 B estourava e corrompia o TCB
   * da housekeepTask (logo abaixo na heap). No voo o debug não é compilado e a pilha fica como antes. */
  .stack_size = 256 * 4,
#else
  .stack_size = 128 * 4,
#endif
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
#if MONITOR_USB || ENABLE_DEBUG_UART2
  .stack_size = 256 * 4,   /* debugPrintf formata texto nesta pilha (ver transmitTask) */
#else
  .stack_size = 128 * 4,
#endif
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for detumblingTask */
osThreadId_t detumblingTaskHandle;
const osThreadAttr_t detumblingTask_attributes = {
  .name = "detumblingTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for missionTask */
osThreadId_t missionTaskHandle;
const osThreadAttr_t missionTask_attributes = {
  .name = "missionTask",
  .stack_size = 256 * 4,
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
#if !ADCS_NA_USART2
UART_HandleTypeDef huart4;          /* bancada: ADCS na UART4 (A0/A1 do Nucleo) */
#define ADCS_HUART huart4
#else
#define ADCS_HUART huart2           /* PCB: ADCS na USART2 */
#endif
/* Flags da task Detumbling (gerente do ADCS); bits 0 e 1 = TASK_START/STOP_FLAG do detumbling */
#define ADCS_AZ_INICIA_FLAG  (1U << 2)
#define ADCS_AZ_PARA_FLAG    (1U << 3)
#define ADCS_AZ_RX_FLAG      (1U << 4)
#define ADCS_EST_RX_FLAG     (1U << 5)
static uint8_t adcsRxByte;
static AdcsAzParser adcsParser;
static volatile char adcsAzIsr[ADCS_AZ_LEN + 1];   /* último azimute completo (ISR -> task) */
static volatile AdcsEstado adcsEstIsr;              /* último estado "SDBF!" (ISR -> task) */
static char g_azimute[ADCS_AZ_LEN + 1] = "";       /* último azimute, para a telemetria */
static volatile uint32_t g_azimuteTick = 0U;
static uint8_t usart1RxBuffer;
static volatile uint32_t usart1LastError;
static volatile uint32_t usart1ErrorCount;
static volatile bool usart1RestartRequired;
static volatile bool antennaDeployed;
static volatile uint32_t usart1UltimaAtividade;   /* tick do último byte/erro na USART1 */
static volatile uint8_t usart1Candidato;          /* byte isolado à espera do silêncio depois dele */
static volatile bool usart1CandidatoValido;
static volatile uint32_t usart1RajadasDescartadas;
#if TC_ON_USART2
static uint8_t usart2RxBuffer;
static volatile uint8_t pendingTelecommand;
#endif
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
static void adcsUartInit(void);
static void handleTelecommand(uint8_t tc, SatMode_t *mode,
		bool *deployInProgress, volatile bool *deployCompleted,
		bool *missionActive);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch) {
#if ENABLE_DEBUG_UART2
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 10);
#endif
	return ch;
}

static void debugPrintf(const char *format, ...)
{
#if MONITOR_USB
	{
		va_list a;
		va_start(a, format);
		monitor_usb_vprintf(format, a);
		va_end(a);
	}
#endif
#if ENABLE_DEBUG_UART2
	va_list args;
	va_start(args, format);
	vprintf(format, args);
	va_end(args);
#else
	(void)format;
#endif
}

/* Resumo legível do quadro 'T' no debug (o printf do newlib-nano não tem float). */
static const char *fmt2(char *b, float v)   /* "-12.34" */
{
	int32_t x = (int32_t)(v * 100.0f + (v < 0.0f ? -0.5f : 0.5f));
	uint32_t m = (uint32_t)(x < 0 ? -x : x);
	(void)snprintf(b, 12, "%s%lu.%02lu", x < 0 ? "-" : "", (unsigned long)(m / 100U), (unsigned long)(m % 100U));
	return b;
}
static void debugTelemetria(const Telemetry_t *t)
{
#if ENABLE_DEBUG_UART2 || MONITOR_USB
	/* static: só a transmitTask chama, e a pilha dela é de 512 B */
	static Telemetry_t c;
	static char v13[12], i13[12], v5[12], i5[12], tb[12], te[12], az[sizeof c.angulo_apontamento + 1];
	c = *t;   /* cópia: Telemetry_t é packed */
	memcpy(az, c.angulo_apontamento, sizeof c.angulo_apontamento);
	az[sizeof c.angulo_apontamento] = '\0';
	debugPrintf("[%08lu] tm %02u:%02u:%02u bat1=%umV/%u%% bat2=%umV/%u%% 13v8=%sV/%sA 5v=%sV/%sA "
			"chg1=%02X/%02X chg2=%02X/%02X tbat=%sC test=%sC deploy=%u az=%s\r\n",
			osKernelGetTickCount(), c.hora, c.minuto, c.segundo,
			(unsigned)c.tensao_bateria_1, (unsigned)c.carga_bateria_1, (unsigned)c.tensao_bateria_2, (unsigned)c.carga_bateria_2,
			fmt2(v13, c.tensao_13v8), fmt2(i13, c.corrente_13v8), fmt2(v5, c.tensao_5v), fmt2(i5, c.corrente_5v),
			c.status_carregador_1, c.falha_carregador_1, c.status_carregador_2, c.falha_carregador_2,
			fmt2(tb, c.temperatura_bateria), fmt2(te, c.temperatura_estrutura), c.deploy_antena, az[0] ? az : "-");
#else
	(void)t;
#endif
}

static void handleTelecommand(uint8_t tc, SatMode_t *mode,
		bool *deployInProgress, volatile bool *deployCompleted,
		bool *missionActive)
{
	if ((uint8_t)(tc & ~((uint8_t)TC_VALID_MASK)) != 0U) {
		debugPrintf("[%08lu] rx rejected: invalid mask 0x%02X\r\n",
				osKernelGetTickCount(), tc);
		return;
	}

	// Bloqueia qualquer troca de modo enquanto o deploy estiver fisicamente em curso
	if (*deployInProgress) {
		debugPrintf("[%08lu] rx ignored: deploy in progress (tc=0x%02X)\r\n",
				osKernelGetTickCount(), tc);
		return;
	}

	// 1. Prioridade máxima: Deploy (bit 1 - 0x02)
	if (((tc & TC_BIT_DEPLOY) != 0U) && !(*deployCompleted)) {
		*deployInProgress = true;
		osThreadFlagsSet(deployTaskHandle, TASK_START_FLAG);
		debugPrintf("[%08lu] rx accepted: deploy\r\n",
				osKernelGetTickCount());
		return;
	}

	// 2. Detumbling (bit 2 - 0x04) - Prioridade sobre Pointing
	if ((tc & TC_BIT_DETUMBLING) != 0U) {
		if (*mode != SAT_MODE_DETUMBLING) {
			*mode = SAT_MODE_DETUMBLING;
			osThreadFlagsSet(detumblingTaskHandle, TASK_START_FLAG);
			debugPrintf("[%08lu] rx accepted: detumbling ON\r\n",
					osKernelGetTickCount());
		}
	} else if (*mode == SAT_MODE_DETUMBLING) {
		*mode = SAT_MODE_IDLE;
		osThreadFlagsSet(detumblingTaskHandle, TASK_STOP_FLAG);
		debugPrintf("[%08lu] rx accepted: detumbling STOP\r\n",
				osKernelGetTickCount());
	}

	// 3. Pointing (bit 3 - 0x08) - Só opera se Detumbling estiver inativo
	if (((tc & TC_BIT_POINTING) != 0U) && ((tc & TC_BIT_DETUMBLING) == 0U)) {
		if (*mode != SAT_MODE_POINTING) {
			*mode = SAT_MODE_POINTING;
			osThreadFlagsSet(detumblingTaskHandle, ADCS_AZ_INICIA_FLAG);   /* ADCS: comando 3 */
			debugPrintf("[%08lu] rx accepted: pointing ON\r\n",
					osKernelGetTickCount());
		}
	} else if (*mode == SAT_MODE_POINTING) {
		*mode = SAT_MODE_IDLE;
		osThreadFlagsSet(detumblingTaskHandle, ADCS_AZ_PARA_FLAG);        /* ADCS: comando 4 */
		debugPrintf("[%08lu] rx accepted: pointing STOP\r\n",
				osKernelGetTickCount());
	}

	// 4. Mission / Carga Útil (bit 0 - 0x01)
	if ((tc & TC_BIT_MISSION) != 0U) {
		if (!(*missionActive)) {
			*missionActive = true;
			if (*mode == SAT_MODE_IDLE) {
				*mode = SAT_MODE_MISSION;
			}
			osThreadFlagsSet(missionTaskHandle, TASK_START_FLAG);
			debugPrintf("[%08lu] rx accepted: mission ON\r\n",
					osKernelGetTickCount());
		}
	} else if (*missionActive) {
		*missionActive = false;
		osThreadFlagsSet(missionTaskHandle, TASK_STOP_FLAG);
		if (*mode == SAT_MODE_MISSION) {
			*mode = SAT_MODE_IDLE;
		}
		debugPrintf("[%08lu] rx accepted: mission OFF\r\n",
				osKernelGetTickCount());
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
  /* Payload HERMES: recebe 20 x 204 = 4080 B a cada 5 s por DMA na USART3 (PB11). */
  HermesAdsb_Init(&huart3, NULL);
  adcsUartInit();

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
#if MONITOR_USB
  /* Bancada com USB: o OTG FS exige HCLK > 14,2 MHz (RM0351). CPU a 16 MHz e APB1/APB2 /4, então UARTs, I2C
   * e timers continuam vendo os mesmos 4 MHz do voo. */
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_8;
#else
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
#endif
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
#if MONITOR_USB
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;
#else
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
#endif

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
  huart1.Init.BaudRate = 9600;
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
#if ADCS_NA_USART2
  /* PCB: a USART2 é do ADCS (SoftwareSerial do Nano, 9600). O CubeMX gera 115200 (debug do Nucleo). */
  huart2.Init.BaudRate = ADCS_BAUD;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
#endif

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
  /* TTC_BUSY: entrada com pull-down (sem o TT&C ligado, lê "livre") */
  GPIO_InitTypeDef busy = {0};
  busy.Pin = TTC_BUSY_PIN;
  busy.Mode = GPIO_MODE_INPUT;
  busy.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(TTC_BUSY_PORT, &busy);

  /* Deploy: CTRL_ANT_1 = PA15, CTRL_ANT_2 = PC10 (saídas, começam em baixo);
   * MT_1 = PC12, MT_2 = PC11 (entradas com pull-up: o botão fecha para o GND enquanto o fio está inteiro) */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
  GPIO_InitTypeDef ctrl = {0};
  ctrl.Mode = GPIO_MODE_OUTPUT_PP;
  ctrl.Pull = GPIO_PULLDOWN;
  ctrl.Speed = GPIO_SPEED_FREQ_LOW;
  ctrl.Pin = GPIO_PIN_15;
  HAL_GPIO_Init(GPIOA, &ctrl);
  ctrl.Pin = GPIO_PIN_10;
  HAL_GPIO_Init(GPIOC, &ctrl);
  GPIO_InitTypeDef mt = {0};
  mt.Pin = GPIO_PIN_11 | GPIO_PIN_12;
  mt.Mode = GPIO_MODE_INPUT;
  mt.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &mt);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void initializeEverything(BME280_HandleTypeDef *BMEBat) {
	/* Relógio zerado a cada boot = tempo desde que o OBC ligou. Dia/mês começam em 1 (0 não é data válida no DS3231):
	 * a telemetria mostra 01/01/00 00:00:00 no boot. */
	Set_Time(0, 0, 0, 1, 1, 1, 0);
	BME280_initDefault(BMEBat, hi2c1);
	BME280_setOversampling(BMEBat, BME280_OS_16x, PSSR); // talvez devesse ser ALL aqui ao invés de PSSR
	eps_init(&hi2c1);   /* MUX PCA9548A + MP2733/MAX17048/INA3221 (como no board_obc) */
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uint32_t agora = osKernelGetTickCount();
        if ((agora - usart1UltimaAtividade) >= TC_SILENCIO_MS)
        {
            usart1Candidato = usart1RxBuffer;      /* silêncio antes: candidato a telecomando */
            usart1CandidatoValido = true;
        }
        else
        {
            if (usart1CandidatoValido) { usart1RajadasDescartadas++; }
            usart1CandidatoValido = false;         /* rajada: nenhum byte dela é telecomando */
        }
        usart1UltimaAtividade = agora;
        (void)osThreadFlagsSet(receiveTaskHandle, USART1_ATIVIDADE_FLAG);

        if (HAL_UART_Receive_IT(&huart1, &usart1RxBuffer, 1U) != HAL_OK)
        {
            usart1LastError = HAL_UART_GetError(&huart1);
            usart1ErrorCount++;
            usart1RestartRequired = true;
            (void)osThreadFlagsSet(receiveTaskHandle, USART1_ERROR_FLAG);
        }
    }
#if TC_ON_USART2
    else if (huart->Instance == USART2)
    {
        /* Same path as a USART1 telecommand. Both IRQs share priority 5,
         * so they never preempt each other on pendingTelecommand. */
        /* Monitor serial: '0'..'9', 'a'..'f', 'A'..'F' viram o telecomando 0x0..0xF; o resto (CR, LF,
         * espaço) é ignorado. Não aceita bytes crus: LF (0x0A) e CR (0x0D) seriam comandos válidos. */
        uint8_t c = usart2RxBuffer, lc = (uint8_t)(c | 0x20U);
        int16_t tc = (c >= '0' && c <= '9') ? (int16_t)(c - '0')
                   : (lc >= 'a' && lc <= 'f') ? (int16_t)(lc - 'a' + 10) : -1;
        (void)HAL_UART_Receive_IT(&huart2, &usart2RxBuffer, 1U);
        if (tc >= 0)
        {
            pendingTelecommand = (uint8_t)tc;
            (void)osThreadFlagsSet(receiveTaskHandle, USART1_RX_FLAG);
        }
    }
#endif
    else if (huart->Instance == HERMES_ADSB_UART_INSTANCE)
    {
        HermesAdsb_RxCpltCallback(huart);
    }
    else if (huart->Instance == ADCS_HUART.Instance)
    {
        char az[ADCS_AZ_LEN + 1];
        AdcsEstado est;
        AdcsMsgTipo tipo = adcs_feed(&adcsParser, adcsRxByte, az, &est);
        if (tipo == ADCS_MSG_AZIMUTE)
        {
            for (uint32_t k = 0U; k <= ADCS_AZ_LEN; k++) { adcsAzIsr[k] = az[k]; }
            (void)osThreadFlagsSet(detumblingTaskHandle, ADCS_AZ_RX_FLAG);
        }
        else if (tipo == ADCS_MSG_ESTADO)
        {
            adcsEstIsr.detumbling = est.detumbling;
            adcsEstIsr.busca = est.busca;
            adcsEstIsr.falha = est.falha;
            (void)osThreadFlagsSet(detumblingTaskHandle, ADCS_EST_RX_FLAG);
        }
        (void)HAL_UART_Receive_IT(&ADCS_HUART, &adcsRxByte, 1U);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        usart1LastError = HAL_UART_GetError(huart);
        usart1ErrorCount++;
        usart1UltimaAtividade = osKernelGetTickCount();   /* erro de quadro = lixo na linha */
        usart1CandidatoValido = false;

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
#if TC_ON_USART2
    else if (huart->Instance == USART2)
    {
        /* Overrun aborts the reception (RxState back to READY); framing and
         * noise errors keep it running. */
        if (huart->RxState == HAL_UART_STATE_READY)
        {
            (void)HAL_UART_Receive_IT(&huart2, &usart2RxBuffer, 1U);
        }
    }
#endif
    else if (huart->Instance == HERMES_ADSB_UART_INSTANCE)
    {
        HermesAdsb_ErrorCallback(huart);
    }
    else if (huart->Instance == ADCS_HUART.Instance)
    {
        /* overrun/ruído: o HAL aborta a recepção; rearma (o parser se realinha sozinho) */
        if (huart->RxState == HAL_UART_STATE_READY)
        {
            (void)HAL_UART_Receive_IT(&ADCS_HUART, &adcsRxByte, 1U);
        }
    }
}

/* ---------------------------------------------------------------- payload ---
 * O Raspberry (HERMES Simples) é ligado e desligado por comando na própria UART
 * (USART3 TX = PB10 -> RX0, pino 10 do Pi), 4 bytes: A5 5A cmd ~cmd, com
 * cmd = 01 LIGAR e 00 DESLIGAR. Enquanto a missão estiver pedida, o LIGAR é
 * reenviado a cada 2 s: se o Pi reiniciar ou perder o comando, ele volta sozinho.
 * A confirmação são os blocos de 4080 B chegando (não há ACK). */
#define PLD_CMD_LIGAR        0x01U
#define PLD_CMD_DESLIGAR     0x00U
#define PLD_CMD_REPETE_MS    2000U    /* reenvio do LIGAR durante a missão */
#define PLD_CMD_OFF_VEZES    3U       /* DESLIGAR vai 3 vezes, 100 ms entre elas */
#define PLD_DOWNLINK_MS      5000U    /* um quadro 'P' ao TT&C a cada 5 s */
#define PLD_SEM_DADOS_MS     20000U   /* aviso: missão ligada e nenhum bloco (o dump1090 leva até 15 s) */

/* Última leitura do TCS (controle térmico das baterias, no Pi), para a telemetria.
 * Escrita pela task Mission, lida pelo housekeeping (palavras de 32 bits: atômicas no M4). */
static volatile int32_t g_tcsMinimaX100 = PLD_TCS_TEMP_INVALIDA;
static volatile uint32_t g_tcsTick = 0U;
#define TCS_VALIDADE_MS 15000U

static void payloadComando(uint8_t cmd)
{
	uint8_t q[4] = { 0xA5U, 0x5AU, cmd, (uint8_t)(cmd ^ 0xFFU) };
	(void)HAL_UART_Transmit(&huart3, q, sizeof(q), 50);
}

static volatile uint32_t ttcBusyEsgotado;   /* quadros mandados com o TT&C ainda ocupado (diagnóstico) */
static volatile uint32_t ttcZDescartados;   /* azimutes 'Z' não mandados porque o TT&C estava ocupado */

static bool ttcOcupado(void) { return HAL_GPIO_ReadPin(TTC_BUSY_PORT, TTC_BUSY_PIN) == GPIO_PIN_SET; }

/* Chamar com o uartMutex: espera o TT&C ficar livre antes de começar um quadro. */
static void ttcEsperaLivre(void)
{
	uint32_t t0 = osKernelGetTickCount();
	while (ttcOcupado()) {
		if ((osKernelGetTickCount() - t0) >= TTC_BUSY_ESPERA_MS) { ttcBusyEsgotado++; return; }
		osDelay(2);
	}
}

/* Manda o quadro 'P' ao TT&C (USART1), sob o mesmo mutex da telemetria. */
static void payloadEnviaDownlink(const uint8_t *frame, size_t len)
{
	osMutexAcquire(uartMutexHandle, osWaitForever);
	ttcEsperaLivre();
	HAL_UART_Transmit(&huart1, (uint8_t *)frame, (uint16_t)len, 2000);
#if defined(BANCADA_ESPELHO) && BANCADA_ESPELHO
	/* Bancada: espelha o quadro na USART2 (ST-Link) para o PC ver os aviões. */
	HAL_UART_Transmit(&huart2, (uint8_t *)frame, (uint16_t)len, 200);
#endif
	osMutexRelease(uartMutexHandle);
}

/* ------------------------------------------------------------------- ADCS ---
 * Comandos de 1 byte (1/2 detumbling, 3/4 azimute). O SoftwareSerial do Nano é
 * half-duplex e pode perder um byte enquanto transmite o azimute (~5 ms/s): cada
 * comando vai 2 vezes, 20 ms depois. Repetir é inofensivo (start/stop idempotentes
 * no Nano). NÃO se reenvia periodicamente: se o ADCS desligar o motor por proteção
 * (sobrevelocidade), o OBC não deve religar sozinho. */
static void adcsUartInit(void)
{
#if !ADCS_NA_USART2
	RCC_PeriphCLKInitTypeDef clk = {0};
	GPIO_InitTypeDef gpio = {0};
	clk.PeriphClockSelection = RCC_PERIPHCLK_UART4;
	clk.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&clk) != HAL_OK) { Error_Handler(); }
	__HAL_RCC_UART4_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;            /* PA0 = UART4_TX (A0), PA1 = UART4_RX (A1) */
	gpio.Mode = GPIO_MODE_AF_PP;
	gpio.Pull = GPIO_PULLUP;
	gpio.Speed = GPIO_SPEED_FREQ_LOW;
	gpio.Alternate = GPIO_AF8_UART4;
	HAL_GPIO_Init(GPIOA, &gpio);
	huart4.Instance = UART4;
	huart4.Init.BaudRate = ADCS_BAUD;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart4) != HAL_OK) { Error_Handler(); }
	HAL_NVIC_SetPriority(UART4_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(UART4_IRQn);
#else
	HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(USART2_IRQn);
#endif
	adcs_az_init(&adcsParser);
	(void)HAL_UART_Receive_IT(&ADCS_HUART, &adcsRxByte, 1U);
}

/* Reset por hardware do Nano (pino RST, ativo em baixo, pull-up de 10 kΩ no próprio Nano).
 * Bancada: D7 = PA8, saída open-drain (o PA8 tolera os 5 V do pull-up). Na PCB do OBC ainda não
 * há essa linha no conector: só o reset por comando (5). */
#if defined(BANCADA_ADCS_RESET) && BANCADA_ADCS_RESET
#define ADCS_RST_PORT GPIOA
#define ADCS_RST_PIN  GPIO_PIN_8
#endif

static void adcsResetInit(void)
{
#ifdef ADCS_RST_PIN
	GPIO_InitTypeDef g = {0};
	__HAL_RCC_GPIOA_CLK_ENABLE();
	HAL_GPIO_WritePin(ADCS_RST_PORT, ADCS_RST_PIN, GPIO_PIN_SET);   /* solto */
	g.Pin = ADCS_RST_PIN;
	g.Mode = GPIO_MODE_OUTPUT_OD;
	g.Pull = GPIO_NOPULL;
	g.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(ADCS_RST_PORT, &g);
#endif
}

static int adcsResetHardware(void)
{
#ifdef ADCS_RST_PIN
	HAL_GPIO_WritePin(ADCS_RST_PORT, ADCS_RST_PIN, GPIO_PIN_RESET);
	osDelay(pdMS_TO_TICKS(50U));
	HAL_GPIO_WritePin(ADCS_RST_PORT, ADCS_RST_PIN, GPIO_PIN_SET);
	return 1;
#else
	return 0;
#endif
}

static void adcsComando(uint8_t cmd)
{
	(void)HAL_UART_Transmit(&ADCS_HUART, &cmd, 1U, 20);
	osDelay(pdMS_TO_TICKS(20U));
	(void)HAL_UART_Transmit(&ADCS_HUART, &cmd, 1U, 20);
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
	bool missionActive = false;

#if MONITOR_USB
	/* bancada na PCB: porta COM virtual na USB do STM32 (espera até 15 s o terminal abrir) */
	if (monitor_usb_init()) {
		debugPrintf("[%08lu] monitor USB: digite o telecomando em hexa (0-f) nesta porta\r\n", osKernelGetTickCount());
	}
#endif
	osTimerStart(HkTickTimerHandle, pdMS_TO_TICKS(HK_PERIODO_MS));
	/* primeira telemetria logo no boot (sem esperar 60 s): mostra ao solo que o OBC está vivo */
	osThreadFlagsSet(housekeepTaskHandle, TASK_START_FLAG);
	debugPrintf("[%08lu] main ready, mode=%u heap_livre=%u\r\n",
			osKernelGetTickCount(), (unsigned int)mode, (unsigned int)xPortGetFreeHeapSize());

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
					&deployInProgress, &antennaDeployed, &missionActive);
			break;

		case EVT_TX_DONE:
			debugPrintf("[%08lu] tx done\r\n", evt.timestamp);
			break;

		case EVT_MISSION_DONE:
			missionActive = false;
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
				/* data.telecommand: bit 0 = antena 1 aberta, bit 1 = antena 2 aberta. Só conta como feito com
				 * as duas; senão um novo TC de deploy tenta de novo (as que já abriram são puladas). */
				deployInProgress = false;
				antennaDeployed = (evt.data.telecommand == 0x03U);
				debugPrintf("[%08lu] deploy done: ant1=%u ant2=%u\r\n", evt.timestamp,
						(unsigned int)(evt.data.telecommand & 1U), (unsigned int)((evt.data.telecommand >> 1) & 1U));
			}
			break;

		case EVT_POINTING_DONE:
			if (mode == SAT_MODE_POINTING) {
				mode = SAT_MODE_IDLE;
				debugPrintf("[%08lu] pointing done, mode=%u\r\n",
						evt.timestamp, (unsigned int)mode);
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
	static Telemetry_t telemetryPacket = {0};
	static BME280_HandleTypeDef BMEBat;
#if MONITOR_USB
	monitor_usb_espera();
#endif
	initializeEverything(&BMEBat);
	/* o que respondeu no I2C da EPS (bits de eps.h), depois do eps_init() */
	{
		uint8_t e = eps_presentes();
		debugPrintf("[%08lu] eps presentes=0x%02X mux=%u mp1=%u mp2=%u max1=%u max2=%u ina1=%u ina2=%u\r\n",
				osKernelGetTickCount(), e, !!(e & EPS_TEM_MUX), !!(e & EPS_TEM_MP1), !!(e & EPS_TEM_MP2),
				!!(e & EPS_TEM_MAX1), !!(e & EPS_TEM_MAX2), !!(e & EPS_TEM_INA1), !!(e & EPS_TEM_INA2));
	}
  /* Infinite loop */
  for(;;)
  {
	  /* Acorda a cada EPS_KEEPALIVE_MS para reaplicar os limites dos carregadores (watchdog do MP2733);
	   * a leitura completa continua a cada 60 s (HkTickTimer) ou no boot. */
	  uint32_t hkFlags = osThreadFlagsWait(TASK_START_FLAG, osFlagsWaitAny, pdMS_TO_TICKS(EPS_KEEPALIVE_MS));
	  if (hkFlags == (uint32_t)osFlagsErrorTimeout) {
		  eps_keepalive();
		  continue;
	  }
	  if (hkFlags & 0x80000000U) {   /* outro erro de osThreadFlagsWait */
		  continue;
	  }
	  BME280_readSingleShot(&BMEBat);

	  telemetryPacket.temperatura_estrutura = BMEBat.READ.TEMP;
	  /* Bateria: a sonda mais fria do TCS (o limite da prova é a bateria acima de 0 °C);
	   * sem TCS recente, cai no BME280 como antes. */
	  int32_t tcsMin = g_tcsMinimaX100;
	  uint32_t tcsTick = g_tcsTick;
	  if (tcsTick != 0U && tcsMin != PLD_TCS_TEMP_INVALIDA
			  && (osKernelGetTickCount() - tcsTick) < TCS_VALIDADE_MS) {
		  telemetryPacket.temperatura_bateria = (float)tcsMin / 100.0f;
	  } else {
		  telemetryPacket.temperatura_bateria = BMEBat.READ.TEMP;
	  }
	  telemetryPacket.umidade = BMEBat.READ.HMDT;
	  telemetryPacket.pressao = BMEBat.READ.PSSR;
	  /* DS3231: o 4º registro é o dia da SEMANA e o 5º o dia do MÊS (antes ia NULL no 5º: escrita no endereço 0
	   * e o dia da semana no campo dia). Sem resposta do RTC no I2C, os campos vão zerados. */
	  uint8_t diaSemana;
	  Get_Time(&telemetryPacket.segundo, &telemetryPacket.minuto, &telemetryPacket.hora, &diaSemana, &telemetryPacket.dia, &telemetryPacket.mes, &telemetryPacket.ano);
	  telemetryPacket.deploy_antena = antennaDeployed ? 1U : 0U;
	  eps_ler(&telemetryPacket);   /* tensões, correntes, carregadores e baterias */
	  /* último azimute recebido do ADCS ("270.5" + '\0'); vazio se nunca chegou */
	  memcpy(telemetryPacket.angulo_apontamento, g_azimute, sizeof(telemetryPacket.angulo_apontamento));

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

	  osMutexAcquire(uartMutexHandle, osWaitForever);
	  ttcEsperaLivre();
	  uint8_t marker = 'T';
	  HAL_UART_Transmit(&huart1, &marker, 1, 100);
	  HAL_UART_Transmit(&huart1, (uint8_t*) telemetryPacket, sizeof(Telemetry_t), 1000);
	  osMutexRelease(uartMutexHandle);
	  debugTelemetria(telemetryPacket);

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

#if TC_ON_USART2
    __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_PEF | UART_CLEAR_FEF
            | UART_CLEAR_NEF | UART_CLEAR_OREF);
    __HAL_UART_SEND_REQ(&huart2, UART_RXDATA_FLUSH_REQUEST);

    if (HAL_UART_Receive_IT(&huart2, &usart2RxBuffer, 1U) != HAL_OK)
    {
        Error_Handler();
    }

    debugPrintf("[%08lu] monitor: digite o telecomando em hexa (0-f) nesta porta\r\n",
            osKernelGetTickCount());
#endif

    for (;;)
    {
        uint32_t espera = osWaitForever;
        if (usart1CandidatoValido)
        {
            uint32_t passou = osKernelGetTickCount() - usart1UltimaAtividade;
            espera = (passou >= TC_SILENCIO_MS) ? 0U : (TC_SILENCIO_MS - passou);
        }
        flags = (espera == 0U) ? 0U
                : osThreadFlagsWait(USART1_RX_FLAG | USART1_ERROR_FLAG | USART1_ATIVIDADE_FLAG,
                        osFlagsWaitAny, espera);

        /* Candidato com TC_SILENCIO_MS de silêncio depois dele: entrega como telecomando. */
        if (usart1CandidatoValido
                && (osKernelGetTickCount() - usart1UltimaAtividade) >= TC_SILENCIO_MS)
        {
            Event_t evt = { .type = EVT_RX_DONE, .timestamp = osKernelGetTickCount() };
            bool entregar = false;
            __disable_irq();   /* reconfere aqui: um byte pode ter chegado depois do teste acima */
            if (usart1CandidatoValido
                    && (osKernelGetTickCount() - usart1UltimaAtividade) >= TC_SILENCIO_MS)
            {
                evt.data.telecommand = usart1Candidato;
                usart1CandidatoValido = false;
                entregar = true;
            }
            __enable_irq();
            if (entregar)
            {
                (void)osMessageQueuePut(eventQueueHandle, &evt, 0U, osWaitForever);
            }
        }

        if (flags == 0U || (flags & osFlagsError) != 0U)
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

        #if TC_ON_USART2
          if ((flags & USART1_RX_FLAG) != 0U)
          {
              Event_t evt = {
                  .type = EVT_RX_DONE,
                  .timestamp = osKernelGetTickCount(),
                  .data.telecommand = pendingTelecommand
              };

              osMessageQueuePut(eventQueueHandle, &evt, 0U, osWaitForever);
          }
        #endif
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
  for(;;)
  {
	  static const struct { GPIO_TypeDef *cp; uint16_t cpin; GPIO_TypeDef *mp; uint16_t mpin; } ant[2] = {
		  { GPIOA, GPIO_PIN_15, GPIOC, GPIO_PIN_12 },   /* antena 1: CTRL_ANT_1 / MT_1 */
		  { GPIOC, GPIO_PIN_10, GPIOC, GPIO_PIN_11 },   /* antena 2: CTRL_ANT_2 / MT_2 */
	  };
	  osThreadFlagsWait(TASK_START_FLAG, osFlagsWaitAny, osWaitForever);
	  uint8_t abertas = 0U;
	  for (uint32_t i = 0U; i < 2U; i++) {
		  if (HAL_GPIO_ReadPin(ant[i].mp, ant[i].mpin) == GPIO_PIN_SET) {
			  abertas |= (uint8_t)(1U << i);
			  debugPrintf("[%08lu] deploy antena %lu: ja aberta, nao queima\r\n", osKernelGetTickCount(), i + 1U);
			  continue;
		  }
		  uint32_t t0 = osKernelGetTickCount();
		  HAL_GPIO_WritePin(ant[i].cp, ant[i].cpin, GPIO_PIN_SET);
		  while (HAL_GPIO_ReadPin(ant[i].mp, ant[i].mpin) == GPIO_PIN_RESET
				  && (osKernelGetTickCount() - t0) < DEPLOY_QUEIMA_MAX_MS) {
			  osDelay(DEPLOY_POLL_MS);
		  }
		  HAL_GPIO_WritePin(ant[i].cp, ant[i].cpin, GPIO_PIN_RESET);
		  uint32_t ms = osKernelGetTickCount() - t0;
		  if (HAL_GPIO_ReadPin(ant[i].mp, ant[i].mpin) == GPIO_PIN_SET) {
			  abertas |= (uint8_t)(1U << i);
			  debugPrintf("[%08lu] deploy antena %lu: aberta em %lu ms\r\n", osKernelGetTickCount(), i + 1U, ms);
		  } else {
			  debugPrintf("[%08lu] deploy antena %lu: FALHOU (botao ainda fechado apos %lu ms)\r\n",
					  osKernelGetTickCount(), i + 1U, ms);
		  }
	  }
	  Event_t evt = {
		  .type = EVT_DEPLOY_DONE,
		  .timestamp = osKernelGetTickCount(),
		  .data.telecommand = abertas
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
	/* Gerente do ADCS. O detumbling só termina por comando (o ADCS não avisa o fim): o modo
	 * fica em DETUMBLING até o TT&C tirar o bit. O azimute chega 1x/s durante a busca. */
	/* Supervisão: o ADCS responde "SDBF!" depois de cada comando e a cada 2 s. Cada vez que o estado
	 * não chega a tempo, ou não bate com o pedido sem uma falha declarada, o OBC reenvia o comando e
	 * conta 1 falha. Com ADCS_FALHAS_MAX seguidas, reinicia o ADCS (comando 5 + pulso no RST, se
	 * houver), espera ele subir e reenvia o pedido. Se o PRÓPRIO ADCS desligou o motor por proteção
	 * (falha 1-3), o OBC NÃO religa: só reporta; o solo decide (tirar e pôr o bit Detumbling). */
#define ADCS_FALHAS_MAX        5U
#define ADCS_ESTADO_TIMEOUT_MS 5000U    /* heartbeat de 2 s + folga */
#define ADCS_CONFIRMA_MS       1500U    /* prazo para o estado depois de um comando */
#define ADCS_BOOT_MS           5000U    /* bootloader + calibração do giroscópio */
#define ADCS_REINICIO_MIN_MS   60000U   /* no máximo 1 reinício por minuto */
	static uint8_t quadroZ[1U + ADCS_AZ_LEN];
	bool querDet = false, querBusca = false, bloqueadoPorFalha = false;
	AdcsEstado est = {0, 0, 0};
	bool temEstado = false;
	uint32_t ultimoEstado = osKernelGetTickCount(), prazoConfirma = 0U, ultimaFalha = 0U;
	uint32_t ultimoReinicio = 0U, reinicios = 0U, esperaBoot = 0U;
	uint8_t falhas = 0U, falhaAnunciada = 0U;

	adcsResetInit();
	/* OBC acabou de ligar: põe o ADCS num estado conhecido, igual ao do OBC (tudo parado). */
	osDelay(pdMS_TO_TICKS(200U));
	adcsComando(ADCS_CMD_DETUMBLING_PARA);
	adcsComando(ADCS_CMD_AZIMUTE_PARA);
	prazoConfirma = osKernelGetTickCount() + ADCS_CONFIRMA_MS;
	debugPrintf("[%08lu] adcs boot do OBC: tx 2 e 4 (sincroniza)\r\n", osKernelGetTickCount());

	for (;;)
	{
		uint32_t f = osThreadFlagsWait(TASK_START_FLAG | TASK_STOP_FLAG | ADCS_AZ_INICIA_FLAG
				| ADCS_AZ_PARA_FLAG | ADCS_AZ_RX_FLAG | ADCS_EST_RX_FLAG, osFlagsWaitAny, pdMS_TO_TICKS(500U));
		uint32_t agora = osKernelGetTickCount();
		if ((f & osFlagsError) != 0U)
		{
			f = 0U;          /* timeout: só a supervisão abaixo */
		}
		if ((f & TASK_STOP_FLAG) != 0U)
		{
			querDet = false;
			bloqueadoPorFalha = false;
			adcsComando(ADCS_CMD_DETUMBLING_PARA);
			prazoConfirma = agora + ADCS_CONFIRMA_MS;
			debugPrintf("[%08lu] adcs tx 2 (encerra detumbling)\r\n", osKernelGetTickCount());
		}
		if ((f & TASK_START_FLAG) != 0U)
		{
			querDet = true;
			bloqueadoPorFalha = false;
			adcsComando(ADCS_CMD_DETUMBLING_INICIA);
			prazoConfirma = agora + ADCS_CONFIRMA_MS;
			debugPrintf("[%08lu] adcs tx 1 (inicia detumbling)\r\n", osKernelGetTickCount());
		}
		if ((f & ADCS_AZ_PARA_FLAG) != 0U)
		{
			querBusca = false;
			adcsComando(ADCS_CMD_AZIMUTE_PARA);
			prazoConfirma = agora + ADCS_CONFIRMA_MS;
			debugPrintf("[%08lu] adcs tx 4 (encerra azimute)\r\n", osKernelGetTickCount());
		}
		if ((f & ADCS_AZ_INICIA_FLAG) != 0U)
		{
			querBusca = true;
			adcsComando(ADCS_CMD_AZIMUTE_INICIA);
			prazoConfirma = agora + ADCS_CONFIRMA_MS;
			debugPrintf("[%08lu] adcs tx 3 (inicia azimute)\r\n", osKernelGetTickCount());
		}

		if ((f & ADCS_EST_RX_FLAG) != 0U)
		{
			taskENTER_CRITICAL();
			est.detumbling = adcsEstIsr.detumbling;
			est.busca = adcsEstIsr.busca;
			est.falha = adcsEstIsr.falha;
			taskEXIT_CRITICAL();
			temEstado = true;
			ultimoEstado = agora;
			debugPrintf("[%08lu] adcs est det=%u busca=%u falha=%u\r\n", agora,
					(unsigned)est.detumbling, (unsigned)est.busca, (unsigned)est.falha);
			if (est.falha != 0U && querDet && !est.detumbling)
			{
				bloqueadoPorFalha = true;       /* proteção do próprio ADCS: não religar */
				if (falhaAnunciada != est.falha)
				{
					falhaAnunciada = est.falha;
					debugPrintf("[%08lu] adcs falha %u: o ADCS desligou o motor; o OBC nao religa\r\n",
							agora, (unsigned)est.falha);
				}
			}
			bool detOk = (est.detumbling == (querDet && !bloqueadoPorFalha)) || bloqueadoPorFalha;
			bool buscaOk = (est.busca == querBusca);
			if (detOk && buscaOk)
			{
				if (falhas != 0U)
				{
					debugPrintf("[%08lu] adcs ok de novo (zera falhas)\r\n", agora);
				}
				falhas = 0U;
				prazoConfirma = 0U;
			}
			else if (esperaBoot == 0U && (int32_t)(agora - prazoConfirma) >= 0)
			{
				/* o comando não pegou (ou o ADCS reiniciou sozinho): reenvia e conta */
				falhas++;
				ultimaFalha = agora;
				debugPrintf("[%08lu] adcs divergente: pedido det=%u busca=%u, adcs det=%u busca=%u (falha %u/%u)\r\n",
						agora, (unsigned)(querDet && !bloqueadoPorFalha), (unsigned)querBusca,
						(unsigned)est.detumbling, (unsigned)est.busca, (unsigned)falhas, (unsigned)ADCS_FALHAS_MAX);
				if (!detOk) { adcsComando((querDet && !bloqueadoPorFalha) ? ADCS_CMD_DETUMBLING_INICIA : ADCS_CMD_DETUMBLING_PARA); }
				if (!buscaOk) { adcsComando(querBusca ? ADCS_CMD_AZIMUTE_INICIA : ADCS_CMD_AZIMUTE_PARA); }
				prazoConfirma = osKernelGetTickCount() + ADCS_CONFIRMA_MS;
			}
		}

		/* sem resposta: conta 1 falha a cada janela de ADCS_ESTADO_TIMEOUT_MS */
		if (esperaBoot == 0U && (agora - ultimoEstado) >= ADCS_ESTADO_TIMEOUT_MS
				&& (agora - ultimaFalha) >= ADCS_ESTADO_TIMEOUT_MS)
		{
			falhas++;
			ultimaFalha = agora;
			debugPrintf("[%08lu] adcs sem resposta ha %lu s (falha %u/%u)\r\n", agora,
					(unsigned long)((agora - ultimoEstado) / 1000U), (unsigned)falhas, (unsigned)ADCS_FALHAS_MAX);
		}

		/* 5 falhas seguidas: reinicia o ADCS e reenvia o pedido depois do boot dele */
		if (falhas >= ADCS_FALHAS_MAX && esperaBoot == 0U
				&& (ultimoReinicio == 0U || (agora - ultimoReinicio) >= ADCS_REINICIO_MIN_MS))
		{
			reinicios++;
			adcsComando(ADCS_CMD_REINICIA);
			int hw = adcsResetHardware();
			ultimoReinicio = osKernelGetTickCount();
			esperaBoot = ultimoReinicio + ADCS_BOOT_MS;
			falhas = 0U;
			bloqueadoPorFalha = false;
			falhaAnunciada = 0U;
			debugPrintf("[%08lu] adcs reinicia #%lu (%u falhas): cmd 5%s\r\n", ultimoReinicio,
					(unsigned long)reinicios, (unsigned)ADCS_FALHAS_MAX, hw ? " + pulso no RST" : "");
		}
		if (esperaBoot != 0U && (int32_t)(agora - esperaBoot) >= 0)
		{
			esperaBoot = 0U;
			ultimoEstado = agora;                 /* recomeça a contar o heartbeat */
			if (querDet) { adcsComando(ADCS_CMD_DETUMBLING_INICIA); }
			if (querBusca) { adcsComando(ADCS_CMD_AZIMUTE_INICIA); }
			prazoConfirma = osKernelGetTickCount() + ADCS_CONFIRMA_MS;
			debugPrintf("[%08lu] adcs pos-reinicio: reenvia det=%u busca=%u\r\n", agora, (unsigned)querDet, (unsigned)querBusca);
		}
		(void)temEstado;
		if ((f & ADCS_AZ_RX_FLAG) != 0U)
		{
			char az[ADCS_AZ_LEN + 1];
			taskENTER_CRITICAL();
			for (uint32_t k = 0U; k <= ADCS_AZ_LEN; k++) { az[k] = adcsAzIsr[k]; }
			taskEXIT_CRITICAL();
			memcpy(g_azimute, az, sizeof(az));
			g_azimuteTick = osKernelGetTickCount();
			/* ao TT&C na hora, num quadro 'Z' + 5 caracteres (no máximo 1 por segundo) */
			quadroZ[0] = (uint8_t)'Z';
			memcpy(&quadroZ[1], az, ADCS_AZ_LEN);
			/* o 'Z' sai da supervisão do ADCS: não espera o TT&C (o próximo azimute vem em 1 s) */
			if (ttcOcupado()) { ttcZDescartados++; }
			else { payloadEnviaDownlink(quadroZ, sizeof(quadroZ)); }
			debugPrintf("[%08lu] adcs az=%s\r\n", osKernelGetTickCount(), az);
		}
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
	/* Gerente do payload HERMES: liga/desliga o Pi por comando na UART, recebe os blocos de
	 * 4080 B (DMA na USART3), valida e repassa ao TT&C a cada 5 s.
	 * A missão termina quando o solo manda um TC sem o bit Mission (TASK_STOP_FLAG). */
	static Plane_t blocos[HERMES_NUM_PLANES];
	static uint8_t tcsBruto[HERMES_TCS_SIZE];
	static uint8_t quadroH[PLD_TCS_FRAME_SIZE];
	static PlaneDown_t avioes[PLD_DOWN_MAX_PLANES];
	static uint8_t quadro[PLD_DOWN_FRAME_MAX];
	uint32_t tcsRuins = 0U;
	size_t nAvioes = 0U;
	bool temDados = false;
	bool missao = false;
	bool avisouSemDados = false;
	uint32_t ultimoDownlink = 0U;
	uint32_t ultimoComando = 0U;
	uint32_t ultimoBloco = 0U;
	uint32_t inicioMissao = 0U;
	uint32_t resyncsVistos = 0U;
	uint32_t errosVistos = 0U;

	HermesAdsb_SetNotifyTask(osThreadGetId());

#if defined(BANCADA_MISSAO_CONTINUA) && BANCADA_MISSAO_CONTINUA
	/* Bancada: payload ligado desde o boot e o DESLIGAR do TT&C ignorado, para o SDR ouvir sem
	 * parar. Nunca usar em voo (build normal não define BANCADA_MISSAO_CONTINUA). */
	missao = true;
	inicioMissao = ultimoDownlink = ultimoComando = osKernelGetTickCount();
	payloadComando(PLD_CMD_LIGAR);
	debugPrintf("[%08lu] pld ligar (bancada: missao continua)\r\n", osKernelGetTickCount());
#endif

	for (;;)
	{
		uint32_t flags = osThreadFlagsWait(TASK_START_FLAG | TASK_STOP_FLAG | HERMES_ADSB_FLAG,
				osFlagsWaitAny, pdMS_TO_TICKS(HERMES_ADSB_TASK_POLL_MS));
		uint32_t agora = osKernelGetTickCount();

		if ((flags & osFlagsError) == 0U)
		{
			if ((flags & TASK_STOP_FLAG) != 0U)
			{
#if defined(BANCADA_MISSAO_CONTINUA) && BANCADA_MISSAO_CONTINUA
				debugPrintf("[%08lu] pld desligar ignorado (bancada: missao continua)\r\n", osKernelGetTickCount());
			}
			if (0)
			{
#endif
				missao = false;
				for (uint32_t k = 0U; k < PLD_CMD_OFF_VEZES; k++)
				{
					payloadComando(PLD_CMD_DESLIGAR);
					osDelay(pdMS_TO_TICKS(100U));
				}
				debugPrintf("[%08lu] pld desligar (UART cmd 00 x%u)\r\n", osKernelGetTickCount(),
						(unsigned int)PLD_CMD_OFF_VEZES);
			}
			if ((flags & TASK_START_FLAG) != 0U)
			{
				missao = true;
				temDados = false;
				avisouSemDados = false;
				inicioMissao = agora;
				ultimoDownlink = agora;
				ultimoComando = agora;
				payloadComando(PLD_CMD_LIGAR);
				debugPrintf("[%08lu] pld ligar (UART cmd 01, repete a cada %u s)\r\n", agora,
						(unsigned int)(PLD_CMD_REPETE_MS / 1000U));
			}
		}

		if (missao && (agora - ultimoComando) >= PLD_CMD_REPETE_MS)
		{
			payloadComando(PLD_CMD_LIGAR);   /* mantém a missão, como um nível */
			ultimoComando = agora;
		}

		const HermesAdsbStats_t *st = HermesAdsb_Stats();
		while (HermesAdsb_Take(blocos, tcsBruto))
		{
			ultimoBloco = agora;
			if (missao)
			{
				PldCompactStats_t cs;
				nAvioes = pld_compact(blocos, avioes, &cs);
				temDados = true;
				avisouSemDados = false;
				debugPrintf("[%08lu] pld bloco %lu: %u avioes, %u vazios, %u descartados\r\n",
						osKernelGetTickCount(), (unsigned long)st->buffers,
						(unsigned int)cs.validos, (unsigned int)cs.vazios, (unsigned int)cs.descartados);
			}

			/* Trailer térmico do TCS: vai para a telemetria e, inteiro, ao TT&C (quadro 'H'). */
			PldTcs_t tcs;
			if (pld_tcs_decode(tcsBruto, &tcs))
			{
				if (tcs.valido)
				{
					g_tcsMinimaX100 = pld_tcs_minima(&tcs);
					g_tcsTick = agora;
				}
				size_t lenH = pld_build_tcs_frame(tcsBruto, quadroH, sizeof(quadroH));
				if (lenH > 0U)
				{
					payloadEnviaDownlink(quadroH, lenH);
				}
				debugPrintf("[%08lu] tcs t=%d,%d,%d,%d duty=%u,%u sit=%u,%u idade=%u valido=%u\r\n",
						osKernelGetTickCount(), (int)tcs.t_x100[0], (int)tcs.t_x100[1], (int)tcs.t_x100[2],
						(int)tcs.t_x100[3], (unsigned int)tcs.duty[0], (unsigned int)tcs.duty[1],
						(unsigned int)tcs.situacao[0], (unsigned int)tcs.situacao[1],
						(unsigned int)tcs.idade_ds, (unsigned int)tcs.valido);
			}
			else
			{
				tcsRuins++;
				debugPrintf("[%08lu] tcs trailer invalido (%lu)\r\n", osKernelGetTickCount(), (unsigned long)tcsRuins);
			}
		}

		HermesAdsb_CheckTimeout(agora);
		if (st->resyncs != resyncsVistos)
		{
			resyncsVistos = st->resyncs;
			if (missao)
			{
				debugPrintf("[%08lu] pld resync %lu (nenhum bloco em %u s)\r\n", agora,
						(unsigned long)resyncsVistos, (unsigned int)(HERMES_ADSB_TIMEOUT_MS / 1000U));
			}
		}
		if (st->dma_errors != errosVistos)
		{
			errosVistos = st->dma_errors;
			debugPrintf("[%08lu] pld erro uart %lu\r\n", agora, (unsigned long)errosVistos);
		}

		if (missao && !avisouSemDados
				&& (agora - (temDados ? ultimoBloco : inicioMissao)) >= PLD_SEM_DADOS_MS)
		{
			avisouSemDados = true;
			debugPrintf("[%08lu] pld sem dados ha %lu s\r\n", agora,
					(unsigned long)((agora - (temDados ? ultimoBloco : inicioMissao)) / 1000U));
		}

		if (missao && temDados && (agora - ultimoDownlink) >= PLD_DOWNLINK_MS)
		{
			size_t len = pld_build_frame(avioes, nAvioes, quadro, sizeof(quadro));
			if (len > 0U)
			{
				payloadEnviaDownlink(quadro, len);
				debugPrintf("[%08lu] ttc tx P n=%u (%u B)\r\n", osKernelGetTickCount(),
						(unsigned int)nAvioes, (unsigned int)len);
			}
			ultimoDownlink = agora;
		}
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
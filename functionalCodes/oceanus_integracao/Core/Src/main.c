/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *
  * Codigo da equipe Polisat para a missao Oceanus, feita para a competicao CubeDesign
  * Autores: Gabriel, Marcos, Milas, Sofia, Sophia
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BME280.h"
#include "INA3221.h"
#include "DS3231.h"
#include "max17048.h"
#include "uart-astreaus.h"
#include <string.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define STANDBY 0
#define ADCS 1
#define MISSION 2
#define FULL 3
#define SAFEMODE 4
#define THERMALSTRESS 5

#define ONADCS 0
#define ONPAYLOAD 1
#define OFFADCS 2
#define OFFPAYLOAD 3
#define GOSTANDBY  4
#define GOTHERMALSTRESS 5
#define GOFULL 6
/* ONADCS = 000 = 0
 * ONPAYLOAD = 001 = 1
 * OFFADCS = 010 = 2
 * OFFPAYLOAD = 011 = 3
 * GOSTANDBY = 100 = 4
 * GOTHERMALSTRESS = 101 = 5
 * GOFULL = 110 = 6  */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

osThreadId CommandReceiverHandle;
osThreadId SafetyWatchdogHandle;
osThreadId TelemetrySenderHandle;
osThreadId StateMachineHandle;
osThreadId OnOffSubsystemHandle;
/* USER CODE BEGIN PV */

//BME sensor temperature
BME280_HandleTypeDef BMEBat; // temperatura da bateria
BME280_HandleTypeDef BMEExt; //le temperatura da estrutura externa

int8_t valorMinBME = 26;
int8_t valorMAXBME = 27; //valores de referencia
float temperatureBat; //leitura temperatura bateria
float temperatureExt; //leitura temperatura externa
uint8_t temperature; // flag para temperature: "low"==0, "ok"==1 ou "high"==2

//DS sensor tempo
uint8_t  sec,min,hour,day,date,month,year;

//INA sensor tensao
INA3221 ina1;
INA3221 ina2;
//seis canais, 3 em cada ina. Um canal de cada vai ler Vbus e Vshunt. Os outros dois canais vao ler so Vbus
//canal 1 do primeiro ina lendo ambos
float shuntvoltage1channel1;
float busvoltage1channel1;
//canal 1 do segundo ina lendo ambos
float shuntvoltage2channel1;
float busvoltage2channel1;
//canais 2 e 3 do primeiro e segundo ina lendo so Vbus
float busvoltage1channel2;
float busvoltage1channel3;
float busvoltage2channel2;
float busvoltage2channel3;

//MAX sensor tensao battery
MAX17048_HandleTypeDef battGauge;
float valorMinTensaobattery = 3.2; //comparado com o valor do min
float valorMaxTensaobattery = 4.2; //comparado com o valor do max
float soc = 0.0f; //carga -> SOC State of Charge
float vbat = 0.0f; //voltagem da battery
uint8_t battery; // flag para battery: "low"==0 ou "ok"==1 ou "high"==2

uint8_t satelliteSafe = 1;

uint8_t state = STANDBY;
uint8_t previousState;
uint8_t blockSafeMode = 0;

uint8_t ttecCommand[1];
uint8_t tx_buffer_ttec[9];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
void StartCommandReceiver(void const * argument);
void StartSafetyWatchdog(void const * argument);
void StartTelemetrySender(void const * argument);
void StartStateMachine(void const * argument);
void StartOnOffSubsystem(void const * argument);

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  //inicializacao dos sensores
    //max
    MAX17048_Init(&battGauge, &hi2c1);

    //bme
    BME280_initDefault(&BMEBat, hi2c1); //bme da bateria com endereco 76
      BME280_setOversampling(&BMEBat, BME280_OS_16x, PSSR); // amostragem de leitura, tem outras opcoes na biblioteca

      //BME280_initOther(&BMEExt, hi2c1);//bme da estrutura com endereco 77
      //BME280_setOversampling(&BMEExt, BME280_OS_16x, PSSR);

     //ds3231
     Set_Time(00, 00, 19, 3, 24, 2, 26); //quarto parametro é o dia da semana, o quinto o dia do mes

     //ina3221
     INA3221_Begin(&hi2c1, &ina1, 1);
     INA3221_Begin(&hi2c1, &ina2, 1);

     ttecCommand[0] = GOSTANDBY;


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
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of CommandReceiver */
  osThreadDef(CommandReceiver, StartCommandReceiver, osPriorityAboveNormal, 0, 128);
  CommandReceiverHandle = osThreadCreate(osThread(CommandReceiver), NULL);

  /* definition and creation of SafetyWatchdog */
  osThreadDef(SafetyWatchdog, StartSafetyWatchdog, osPriorityHigh, 0, 128);
  SafetyWatchdogHandle = osThreadCreate(osThread(SafetyWatchdog), NULL);

  /* definition and creation of TelemetrySender */
  osThreadDef(TelemetrySender, StartTelemetrySender, osPriorityLow, 0, 128);
  TelemetrySenderHandle = osThreadCreate(osThread(TelemetrySender), NULL);

  /* definition and creation of StateMachine */
  osThreadDef(StateMachine, StartStateMachine, osPriorityRealtime, 0, 128);
  StateMachineHandle = osThreadCreate(osThread(StateMachine), NULL);

  /* definition and creation of OnOffSubsystem */
  osThreadDef(OnOffSubsystem, StartOnOffSubsystem, osPriorityLow, 0, 128);
  OnOffSubsystemHandle = osThreadCreate(osThread(OnOffSubsystem), NULL);

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
	  //vazio porque tarefas sao executadas com RTOS
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, PAYLOAD_END_Pin|PAYLOAD_START_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ADCS_START_Pin|LED_YELLOW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_RED_Pin|LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : TTC_BUSY_Pin */
  GPIO_InitStruct.Pin = TTC_BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TTC_BUSY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PAYLOAD_END_Pin PAYLOAD_START_Pin */
  GPIO_InitStruct.Pin = PAYLOAD_END_Pin|PAYLOAD_START_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ADCS_START_Pin LED_YELLOW_Pin */
  GPIO_InitStruct.Pin = ADCS_START_Pin|LED_YELLOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_RED_Pin LED_BLUE_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin|LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartCommandReceiver */
/**
  * @brief  Function implementing the CommandReceiver thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartCommandReceiver */
void StartCommandReceiver(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	TT_C_ReceiveData(&huart1, ttecCommand, sizeof(ttecCommand));
    osDelay(1000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSafetyWatchdog */
/**
* @brief Function implementing the SafetyWatchdog thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSafetyWatchdog */
void StartSafetyWatchdog(void const * argument)
{
  /* USER CODE BEGIN StartSafetyWatchdog */
  /* Infinite loop */
  for(;;)
  {
	  //Leitura dos sensores
		BME280_readSingleShot(&BMEBat); //BME280
		//BME280_readSingleShot(&BMEExt); //BME280

		Get_Time(&sec,&min,&hour,&day,&date,&month,&year); //ds3231
		// day = dia da semana, date = dia do mes

		//temperature BME
		temperatureBat = BMEBat.READ.TEMP;
		//temperatureExt = BMEExt.READ.TEMP;
		//Verificação de temperature, muda flag temperature se uma está muito baixa ou alta
		if (temperatureBat < valorMinBME) { //temp mto baixa
			temperature = 0;
			HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, 0);
			HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, 1);
			HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, 0);
		}
		else if (temperatureBat > valorMAXBME) { //temp mto alta
			temperature = 2;
			HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, 1);
			HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, 0);
			HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, 0);
		}
		else {
			temperature = 1; //temperature ok
			HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, 0);
			HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, 0);
			HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, 1);
		}

		battery = 1; //nao estou lendo ainda

		if (temperature == 1 && battery == 1) satelliteSafe = 1;
		else satelliteSafe = 0;

		osDelay(1000);
  }
  /* USER CODE END StartSafetyWatchdog */
}

/* USER CODE BEGIN Header_StartTelemetrySender */
/**
* @brief Function implementing the TelemetrySender thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTelemetrySender */
void StartTelemetrySender(void const * argument)
{
  /* USER CODE BEGIN StartTelemetrySender */

  /* Infinite loop */
  for(;;)
  {
	  //DS3231
	  tx_buffer_ttec[0] = hour; tx_buffer_ttec[1] = min; tx_buffer_ttec[2] = sec;
	  tx_buffer_ttec[3] = date; tx_buffer_ttec[4] = month; tx_buffer_ttec[5] = year;

	  //BME280
	  tx_buffer_ttec[6] = temperatureBat; //parte inteira da temp lida pelo bme
	  tx_buffer_ttec[7] = temperatureBat * 100 - tx_buffer_ttec[6] * 100; //parte decimal
	  tx_buffer_ttec[8] = 'P';

	  TT_C_SendCommand(&huart1, tx_buffer_ttec, sizeof(tx_buffer_ttec)); //funcao que manda para TTeC
    osDelay(3000);

  }
  /* USER CODE END StartTelemetrySender */
}

/* USER CODE BEGIN Header_StartStateMachine */
/**
* @brief Function implementing the StateMachine thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartStateMachine */
void StartStateMachine(void const * argument)
{
  /* USER CODE BEGIN StartStateMachine */

//comandos ttec
//#define ONADCS 0
//#define ONPAYLOAD 1
//#define OFFADCS 2
//#define OFFPAYLOAD 3
//#define GOSTANDBY  4
//#define GOTHERMALSTRESS 5
//#define GOFULL 6

//estados
//#define STANDBY 0
//#define ADCS 1
//#define MISSION 2
//#define FULL 3
//#define SAFEMODE 4
//#define THERMALSTRESS 5

  /* Infinite loop */
  for(;;)
  {
	  previousState = state; //previous state esta sendo atualisado a cada milisegundo -> errado

	  if (!satelliteSafe){ //satelite nao seguro
		  if (state != SAFEMODE) previousState = state;
		  if (!blockSafeMode && state != THERMALSTRESS) state = SAFEMODE; //entrar no safemode
	  }
	  else { //satelite seguro
		  if (state == SAFEMODE) state = previousState;
		  blockSafeMode = 0; //desbloquear safemode, para poder entrar em uma ocasiao de risco novamente
	  }

	  switch (ttecCommand[0]){
	  case ONADCS:
		  if (state == STANDBY) state = ADCS;
		  else if (state==MISSION) state = FULL;
		  break;
	  case ONPAYLOAD:
		  if (state == STANDBY) state = MISSION;
		  else if (state==ADCS) state = FULL;
		  break;
	  case OFFADCS:
		  if (state==ADCS) state=STANDBY;
		  else if (state==FULL) state=MISSION;
		  break;
	  case OFFPAYLOAD:
		  if (state==MISSION) state=STANDBY;
		  else if (state==FULL) state=ADCS;
		  break;
	  case GOTHERMALSTRESS:
		  state = THERMALSTRESS;
		  break;
	  case GOFULL:
		  if (state == STANDBY || state==MISSION||state==ADCS) state = FULL;
		  break;
	  case GOSTANDBY:
		  if (state==SAFEMODE) blockSafeMode = 1;
		  state = STANDBY;

	  default:
	  		  break;
	  }
	osDelay(1500);
  }
  /* USER CODE END StartStateMachine */
}

/* USER CODE BEGIN Header_StartOnOffSubsystem */
/**
* @brief Function implementing the OnOffSubsystem thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartOnOffSubsystem */
void StartOnOffSubsystem(void const * argument)
{
  /* USER CODE BEGIN StartOnOffSubsystem */
  /* Infinite loop */
  for(;;)
  {
	  switch (state){
	  case STANDBY:
		  ADCS_LigaDesliga(0);
		  CargaUtil_End();
		  break;
	  case THERMALSTRESS:
		  ADCS_LigaDesliga(0);
		  CargaUtil_End();
		  break;
	  case ADCS:
		  ADCS_LigaDesliga(1);
		  CargaUtil_End();
		  break;
	  case MISSION:
		  ADCS_LigaDesliga(0);
		  CargaUtil_Start();
		  break;
	  case FULL:
		  ADCS_LigaDesliga(1);
		  CargaUtil_Start();
		  break;
	  }
    osDelay(1000);
  }
  /* USER CODE END StartOnOffSubsystem */
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

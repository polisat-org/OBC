/* USER CODE BEGIN Header */
/**
  ******************************************************************************
- @file           : main.c
- @brief          : Firmware principal - USB CDC, I2C1 direto (BME280, DS3231)
-                   e MUX I2C 0x70 (2x MP2733, 2x MAX17048, 2x INA3221)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include <stdio.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ina3221.h"
#include "max17048.h"
#include "mp2733.h"
#include "ds3231.h"
#include "bme280.h"
#include "usbd_cdc_if.h"
/*#include "telemetryType.h"*/  
#include "dataReadying.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BME_USE_UART        1

/* Acerto da hora do DS3231:
- 0 = nao acerta; so acerta com a hora da compilacao se OSF = 1
- 1 = acerta SEMPRE no boot com a data/hora da compilacao
- 2 = acerta SEMPRE no boot com os valores fixos abaixo */
#define DS_TIME_SOURCE      1

#define DS_FIXED_YEAR       2026
#define DS_FIXED_MONTH      11
#define DS_FIXED_DAY        8
#define DS_FIXED_HOUR       14
#define DS_FIXED_MIN        30
#define DS_FIXED_SEC        0

/* Enderecos I2C */
#define MUX_ADDR            (0x70)
#define INA_ADDR            (0x40)
#define INA_ADDR2           (0x41)

/* Canais mapeados no MUX */
#define CH_BAT1             0   /* MP1 (0x4B) + MAX1 (0x36) */
#define CH_BAT2             1   /* MP2 (0x4B) + MAX2 (0x36) */
#define CH_MP1              CH_BAT1
#define CH_MAX1             CH_BAT1
#define CH_MP2              CH_BAT2
#define CH_MAX2             CH_BAT2
#define CH_INA1             2   /* INA1 (0x40) */
#define CH_INA2             3   /* INA2 (0x41) */

/* Configuracao de corrente de carga/saida do MP2733 (REG05h - ICC[6:0]) */
#define MP2733_REG_ICHG     (0x05)
#define MP2733_ICHG_MASK    (0x7F)
#define MP2733_ICHG_1_0A    (0x19)  /* 25 * 40 mA = 1000 mA */

/* Limite de corrente de entrada (REG00h - IINLIM[5:0]) */
#define MP2733_REG_IINLIM   (0x00)
#define MP2733_IINLIM_MASK  (0x7F)
#define MP2733_IINLIM_1_5A  (0x1C)  /* 100 mA + 28 * 50 mA = 1500 mA */

#define RX_BUF_SIZE         64

/* LED verde de indicacao de boot (PB5) */
#define LED_VERDE_GPIO_Port GPIOB
#define LED_VERDE_Pin       GPIO_PIN_5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Buffers UART */
uint8_t rx_u1[RX_BUF_SIZE];
char    msg_u1[RX_BUF_SIZE] = {0};
volatile uint8_t new_data_u1 = 0;

uint8_t rx_u2[RX_BUF_SIZE];
char    msg_u2[RX_BUF_SIZE] = {0};
volatile uint8_t new_data_u2 = 0;

uint8_t rx_u3[RX_BUF_SIZE];
char    msg_u3[RX_BUF_SIZE] = {0};
volatile uint8_t new_data_u3 = 0;

/* TTC / Telemetria */
telemetryFull t1; /* t2;
uint8_t buffer_envio[51];
uint8_t tamanho1, tamanho2; */

/* BME280 (barramento principal I2C1) */
BME280_t      bme;
BME280_Data_t bme_data;
BME280_Status bme_status = BME280_ERR_I2C;
uint8_t       bme_found  = 0;

/* Variaveis para Live Watch */
volatile float    dbg_temp_c      = 0.0f;
volatile float    dbg_press_hpa   = 0.0f;
volatile float    dbg_hum_pct     = 0.0f;
volatile uint32_t dbg_read_count  = 0;
volatile uint32_t dbg_error_count = 0;

/* DS3231 (barramento principal I2C1) */
DS3231_t      rtc;
DS3231_Time_t rtc_time;
int16_t       rtc_temp_centi = 0;
uint8_t       rtc_found      = 0;

/* MUX I2C e dispositivos downstream */
uint8_t mux_found = 0;

INA3221_Handle ina1;
INA3221_Handle ina2;
uint8_t ina1_ativo = 0;
uint8_t ina2_ativo = 0;

MP2733_HandleTypeDef mp1;
MP2733_HandleTypeDef mp2;
uint8_t mp1_ativo = 0;
uint8_t mp2_ativo = 0;

MAX17048_HandleTypeDef max1;
MAX17048_HandleTypeDef max2;
uint8_t max1_ativo = 0;
uint8_t max2_ativo = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);

/* USER CODE BEGIN PFP */
static void USB_Print(const char *text);

/* MUX e diagnostico I2C */
static HAL_StatusTypeDef MUX_Write(I2C_HandleTypeDef *hi2c, uint8_t data);
HAL_StatusTypeDef MUX_Read(I2C_HandleTypeDef *hi2c, uint8_t *out);
static HAL_StatusTypeDef MUX_SelectChannel(I2C_HandleTypeDef *hi2c, uint8_t channel);
static HAL_StatusTypeDef MUX_DisableAll(I2C_HandleTypeDef *hi2c);
static uint8_t MUX_Detect(I2C_HandleTypeDef *hi2c);
void MUX_ScanAll(I2C_HandleTypeDef *hi2c);
static void I2C1_PinReport(void);
static uint8_t I2C1_ScanBus(void);

/* Sensores diretos (I2C1) */
static uint8_t BME_Detect(void);
static uint8_t RTC_Setup(void);
static void RTC_ReadAndPrint(void);

/* Dispositivos via MUX */
static void MP2733_SetCustomCurrent(I2C_HandleTypeDef *hi2c, uint8_t reg_addr, uint8_t mask, uint8_t value);
static uint8_t MP2733_Setup(MP2733_HandleTypeDef *h, const char *nome);
static void MP2733_ReadAndPrint(MP2733_HandleTypeDef *h, const char *nome, uint8_t *status_out, uint8_t *fault_out);

static uint8_t MAX17048_Setup(MAX17048_HandleTypeDef *h, const char *nome);
static void MAX17048_ReadAndPrint(MAX17048_HandleTypeDef *h, const char *nome, uint16_t *vbat_mv_out, uint16_t *soc_out);

static uint8_t INA3221_SetupChannel(INA3221_Handle *h, uint8_t addr_7bit, const char *nome);
static void INA1_ReadAndPrint(INA3221_Handle *h);
static void INA2_ReadAndPrint(INA3221_Handle *h);

static void Telemetry_InitDefaults(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void USB_Print(const char *text)
{
  uint16_t len = (uint16_t)strlen(text);
  uint32_t t0  = HAL_GetTick();

  while (CDC_Transmit_FS((uint8_t *)text, len) == USBD_BUSY) {
    if ((HAL_GetTick() - t0) > 20U) {
      return;
    }
  }
  HAL_Delay(5);
}

static HAL_StatusTypeDef MUX_Write(I2C_HandleTypeDef *hi2c, uint8_t data)
{
  char buf[80];
  HAL_StatusTypeDef r = HAL_I2C_Master_Transmit(hi2c, MUX_ADDR << 1, &data, 1, 100);
  if (r != HAL_OK) {
    snprintf(buf, sizeof(buf), "MUX falha escrita 0x%02X (err=0x%lX)\r\n",
             data, (unsigned long)HAL_I2C_GetError(hi2c));
    USB_Print(buf);
  }
  return r;
}

HAL_StatusTypeDef MUX_Read(I2C_HandleTypeDef *hi2c, uint8_t *out)
{
  char buf[80];
  HAL_StatusTypeDef r = HAL_I2C_Master_Receive(hi2c, MUX_ADDR << 1, out, 1, 100);
  if (r != HAL_OK) {
    snprintf(buf, sizeof(buf), "MUX falha leitura (err=0x%lX)\r\n",
             (unsigned long)HAL_I2C_GetError(hi2c));
    USB_Print(buf);
  }
  return r;
}

static HAL_StatusTypeDef MUX_SelectChannel(I2C_HandleTypeDef *hi2c, uint8_t channel)
{
  if (channel > 7U) {
    return HAL_ERROR;
  }
  return MUX_Write(hi2c, (uint8_t)(1U << channel));
}

static HAL_StatusTypeDef MUX_DisableAll(I2C_HandleTypeDef *hi2c)
{
  if (!mux_found) {
    return HAL_OK;
  }
  return MUX_Write(hi2c, 0x00U);
}

static uint8_t MUX_Detect(I2C_HandleTypeDef *hi2c)
{
  if (HAL_I2C_IsDeviceReady(hi2c, MUX_ADDR << 1, 3, 50) == HAL_OK) {
    USB_Print("MUX I2C encontrado em 0x70\r\n");
    return 1;
  }
  USB_Print("MUX I2C (0x70) nao respondeu\r\n");
  return 0;
}

void MUX_ScanAll(I2C_HandleTypeDef *hi2c)
{
  char buf[64];
  for (uint8_t ch = 0; ch < 8U; ch++) {
    if (MUX_SelectChannel(hi2c, ch) == HAL_OK) {
      snprintf(buf, sizeof(buf), "MUX CH%u -> ", (unsigned)ch);
      USB_Print(buf);
      I2C1_ScanBus();
    }
  }
  MUX_DisableAll(hi2c);
}
/* USER CODE END 0 */

/**
- @brief  The application entry point.
- @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  HAL_Init();

  /* USER CODE BEGIN Init */
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
  HAL_PWREx_EnableVddUSB();
  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
  /* Pisca LED verde (PB5) ao ligar a placa (3 pulsos = 900 ms) */
  for (uint8_t i = 0; i < 3U; i++) {
    HAL_GPIO_WritePin(LED_VERDE_GPIO_Port, LED_VERDE_Pin, GPIO_PIN_SET);
    HAL_Delay(150);
    HAL_GPIO_WritePin(LED_VERDE_GPIO_Port, LED_VERDE_Pin, GPIO_PIN_RESET);
    HAL_Delay(150);
  }
  HAL_Delay(1100); /* Completa os 2000 ms para enumeracao da USB CDC */

  USB_Print("\r\n=== Sistema OBC Missao Hermes (I2C1 + MUX 0x70) ===\r\n");

  I2C1_PinReport();
  mux_found = MUX_Detect(&hi2c1);
  MUX_DisableAll(&hi2c1);

  /* Scan do barramento principal (esperado: 0x68 DS3231, 0x70 MUX, 0x76/0x77 BME280) */
  USB_Print("Scan I2C1 principal: ");
  I2C1_ScanBus();

  /* 1. Inicializa dispositivos diretos no I2C1 */
  bme_found = BME_Detect();
  if (bme_found) {
    if (bme.chip_id == BME280_CHIP_ID_BME280) {
      USB_Print("Sensor: BME280 (0x60)\r\n");
    } else {
      USB_Print("Sensor: BMP280 (0x58), sem umidade\r\n");
    }
  } else {
    USB_Print("BME280 nao encontrado\r\n");
  }

  rtc_found = RTC_Setup();

  /* 2. Inicializa dispositivos atras do MUX (0x70) */
  if (mux_found) {
    /* Canal 0: MP1 (0x4B) e MAX1 (0x36) */
    if (MUX_SelectChannel(&hi2c1, CH_BAT1) == HAL_OK) {
      mp1_ativo  = MP2733_Setup(&mp1, "MP1 (CH0)");
      max1_ativo = MAX17048_Setup(&max1, "MAX1 (CH0)");
    }

    /* Canal 1: MP2 (0x4B) e MAX2 (0x36) */
    if (MUX_SelectChannel(&hi2c1, CH_BAT2) == HAL_OK) {
      mp2_ativo  = MP2733_Setup(&mp2, "MP2 (CH1)");
      max2_ativo = MAX17048_Setup(&max2, "MAX2 (CH1)");
    }

    /* Canal 2: INA1 (0x40) */
    if (MUX_SelectChannel(&hi2c1, CH_INA1) == HAL_OK) {
      ina1_ativo = INA3221_SetupChannel(&ina1, INA_ADDR, "INA1 (CH2 - 0x40)");
    }

    /* Canal 3: INA2 (0x41) */
    if (MUX_SelectChannel(&hi2c1, CH_INA2) == HAL_OK) {
      ina2_ativo = INA3221_SetupChannel(&ina2, INA_ADDR2, "INA2 (CH3 - 0x41)");
    }

    /* Isola todos os canais do MUX ao fim do boot */
    MUX_DisableAll(&hi2c1);
  }

  /* Limpeza e ativacao das UARTs por interrupcao */
  __HAL_UART_CLEAR_OREFLAG(&huart1);
  __HAL_UART_CLEAR_FEFLAG(&huart1);
  __HAL_UART_CLEAR_NEFLAG(&huart1);
  HAL_UARTEx_ReceiveToIdle_IT(&huart1, rx_u1, RX_BUF_SIZE - 1);

  __HAL_UART_CLEAR_OREFLAG(&huart2);
  __HAL_UART_CLEAR_FEFLAG(&huart2);
  __HAL_UART_CLEAR_NEFLAG(&huart2);
  HAL_UARTEx_ReceiveToIdle_IT(&huart2, rx_u2, RX_BUF_SIZE - 1);

  __HAL_UART_CLEAR_OREFLAG(&huart3);
  __HAL_UART_CLEAR_FEFLAG(&huart3);
  __HAL_UART_CLEAR_NEFLAG(&huart3);
  HAL_UARTEx_ReceiveToIdle_IT(&huart3, rx_u3, RX_BUF_SIZE - 1);

  Telemetry_InitDefaults();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* Garante MUX isolado antes de ler sensores diretos no I2C1 */
    MUX_DisableAll(&hi2c1);

    /* 1. DS3231 (RTC) */
    if (!rtc_found) {
      rtc_found = RTC_Setup();
    }
    if (rtc_found) {
      RTC_ReadAndPrint();
    } else {
      if (t1.segundo < 59) t1.segundo++;
      else t1.segundo = 0;
    }

    /* 2. BME280 */
    if (!bme_found) {
      bme_found = BME_Detect();
    } else {
      bme_status = BME280_ReadForced(&bme, &bme_data);
      if (bme_status == BME280_OK) {
        char buf[112];
        int32_t  t     = bme_data.temp_centi;
        uint32_t t_abs = (uint32_t)((t < 0) ? -t : t);
        uint32_t p     = bme_data.press_pa;
        uint32_t h     = bme_data.hum_centi;

        dbg_temp_c    = (float)bme_data.temp_centi / 100.0f;
        dbg_press_hpa = (float)bme_data.press_pa   / 100.0f;
        dbg_hum_pct   = (float)bme_data.hum_centi  / 100.0f;
        dbg_read_count++;

        t1.temperatura_externa = dbg_temp_c;

        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_10);

        snprintf(buf, sizeof(buf),
                 "BME280: T=%s%lu.%02lu C | P=%lu.%02lu hPa | UR=%lu.%02lu %%\r\n",
                 (t < 0) ? "-" : "",
                 (unsigned long)(t_abs / 100U), (unsigned long)(t_abs % 100U),
                 (unsigned long)(p / 100U),     (unsigned long)(p % 100U),
                 (unsigned long)(h / 100U),     (unsigned long)(h % 100U));
        USB_Print(buf);
      } else {
        dbg_error_count++;
        USB_Print("Erro na leitura do BME280\r\n");
        bme_found = 0;
      }
    }

    /* 3. Leituras via MUX (0x70) */
    if (!mux_found) {
      mux_found = MUX_Detect(&hi2c1);
    }

    if (mux_found) {
      /* Canal 0: Bateria 1 (MP2733 + MAX17048) */
      if ((mp1_ativo || max1_ativo) && MUX_SelectChannel(&hi2c1, CH_BAT1) == HAL_OK) {
        if (mp1_ativo) {
          MP2733_ReadAndPrint(&mp1, "MP1 - CH0", &t1.status_carregador_1, &t1.falha_carregador_1);
        }
        if (max1_ativo) {
          MAX17048_ReadAndPrint(&max1, "MAX1 - CH0", &t1.tensao_bateria_1, &t1.carga_bateria_1);
        }
      }

      /* Canal 1: Bateria 2 (MP2733 + MAX17048) */
      if ((mp2_ativo || max2_ativo) && MUX_SelectChannel(&hi2c1, CH_BAT2) == HAL_OK) {
        if (mp2_ativo) {
          MP2733_ReadAndPrint(&mp2, "MP2 - CH1", &t1.status_carregador_2, &t1.falha_carregador_2);
        }
        if (max2_ativo) {
          MAX17048_ReadAndPrint(&max2, "MAX2 - CH1", &t1.tensao_bateria_2, &t1.carga_bateria_2);
        }
      }

      /* Canal 2: INA1 (0x40 - Tensoes) */
      if (ina1_ativo && MUX_SelectChannel(&hi2c1, CH_INA1) == HAL_OK) {
        INA1_ReadAndPrint(&ina1);
      }

      /* Canal 3: INA2 (0x41 - Correntes e Tensao) */
      if (ina2_ativo && MUX_SelectChannel(&hi2c1, CH_INA2) == HAL_OK) {
        INA2_ReadAndPrint(&ina2);
      }

      /* Fecha todas as chaves do MUX entre amostras */
      MUX_DisableAll(&hi2c1);
    }

    /* 4. Empacotamento e verificacao de telemetria */
    /* tamanho1 = readyData(&t1, buffer_envio);
    tamanho2 = parseData(&t2, buffer_envio);

    if (t1.segundo % 15 == 0) {
      printTelemetry(&t2);
    } */

    /* 5. Tratamento das UARTs */
    if (new_data_u1) {
      new_data_u1 = 0;
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
      USB_Print("UART1 RX: ");
      USB_Print(msg_u1);
    } else {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
    }

    if (new_data_u2) {
      new_data_u2 = 0;
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
      USB_Print("UART2 RX: ");
      USB_Print(msg_u2);
    } else {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    }

    if (new_data_u3) {
      new_data_u3 = 0;
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
      USB_Print("UART3 RX: ");
      USB_Print(msg_u3);
    } else {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
    }

    USB_Print("----------------------------\r\n");
    HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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

  HAL_RCCEx_EnableMSIPLLMode();
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00503D58;
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

  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART1_UART_Init(void)
{
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
}

static void MX_USART2_UART_Init(void)
{
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
}

static void MX_USART3_UART_Init(void)
{
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
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* ---------------------------------- Diagnostico I2C ---------------------- */
static void I2C1_PinReport(void)
{
  static const char modes[4] = {'I', 'O', 'A', 'N'};
  char msg[128];
  int len = snprintf(msg, sizeof(msg), "Pinos GPIOB:");

  for (uint32_t pin = 6U; pin <= 9U; pin++) {
    uint32_t mode = (GPIOB->MODER >> (pin * 2U)) & 0x3U;
    uint32_t af   = (GPIOB->AFR[pin >> 3U] >> ((pin & 7U) * 4U)) & 0xFU;
    uint32_t lvl  = (GPIOB->IDR >> pin) & 1U;

    len += snprintf(msg + len, sizeof(msg) - (size_t)len, " PB%lu=%c",
                    (unsigned long)pin, modes[mode]);
    if (mode == 2U) {
      len += snprintf(msg + len, sizeof(msg) - (size_t)len, "(AF%lu)", (unsigned long)af);
    }
    len += snprintf(msg + len, sizeof(msg) - (size_t)len, "/%lu", (unsigned long)lvl);
  }
  snprintf(msg + len, sizeof(msg) - (size_t)len, " BUSY=%d\r\n",
           (I2C1->ISR & I2C_ISR_BUSY) ? 1 : 0);
  USB_Print(msg);
}

static uint8_t I2C1_ScanBus(void)
{
  char buf[16];
  uint8_t n = 0;

  for (uint8_t a = 0x08; a < 0x78; a++) {
    if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(a << 1), 1, 5) == HAL_OK) {
      snprintf(buf, sizeof(buf), "0x%02X ", a);
      USB_Print(buf);
      n++;
    }
  }
  if (n == 0) {
    USB_Print("nenhum dispositivo");
  }
  USB_Print("\r\n");
  return n;
}

/* ---------------------------------- DS3231 ------------------------------- */
static uint8_t RTC_Setup(void)
{
  DS3231_Time_t t = {0};
  uint8_t osf = 0;
  uint8_t must_set = 0;

  if (DS3231_Init(&rtc, &hi2c1) != DS3231_OK) {
    USB_Print("DS3231 nao encontrado (0x68)\r\n");
    return 0;
  }
  USB_Print("DS3231 encontrado\r\n");

  if (DS3231_OscStopped(&rtc, &osf) != DS3231_OK) {
    return 0;
  }
  (void)osf;

#if DS_TIME_SOURCE == 1
  if (DS3231_FromCompileTime(&t, __DATE__, __TIME__) != DS3231_OK) {
    USB_Print("Falha ao ler data/hora da compilacao\r\n");
    return 0;
  }
  must_set = 1;
#elif DS_TIME_SOURCE == 2
  t.year  = DS_FIXED_YEAR;
  t.month = DS_FIXED_MONTH;
  t.day   = DS_FIXED_DAY;
  t.hour  = DS_FIXED_HOUR;
  t.min   = DS_FIXED_MIN;
  t.sec   = DS_FIXED_SEC;
  t.wday  = 0;
  must_set = 1;
#else
  if (osf) {
    if (DS3231_FromCompileTime(&t, __DATE__, __TIME__) == DS3231_OK) {
      must_set = 1;
    }
  }
#endif

  if (must_set) {
    if (DS3231_SetTime(&rtc, &t) != DS3231_OK) {
      USB_Print("Falha ao acertar hora do DS3231\r\n");
      return 0;
    }
    USB_Print("Hora do DS3231 acertada\r\n");
  }
  return 1;
}

static void RTC_ReadAndPrint(void)
{
  static const char *const wd[8] = {"???", "seg", "ter", "qua", "qui", "sex", "sab", "dom"};
  char buf[96];
  int16_t temp = 0;
  uint16_t temp_abs;

  if ((DS3231_GetTime(&rtc, &rtc_time) != DS3231_OK) ||
      (DS3231_GetTemperature(&rtc, &temp) != DS3231_OK)) {
    USB_Print("Erro ao ler o DS3231\r\n");
    rtc_found = 0;
    return;
  }

  /* Atualiza campos de data/hora da telemetria */
  t1.hora    = rtc_time.hour;
  t1.minuto  = rtc_time.min;
  t1.segundo = rtc_time.sec;
  t1.dia     = rtc_time.day;
  t1.mes     = rtc_time.month;
  t1.ano     = (uint8_t)(rtc_time.year % 100U);

  rtc_temp_centi = temp;
  temp_abs = (uint16_t)((temp < 0) ? -temp : temp);

  snprintf(buf, sizeof(buf),
           "RTC: %04u-%02u-%02u (%s) %02u:%02u:%02u | Temp RTC: %s%u.%02u C\r\n",
           (unsigned)rtc_time.year, (unsigned)rtc_time.month, (unsigned)rtc_time.day,
           wd[(rtc_time.wday >= 1U && rtc_time.wday <= 7U) ? rtc_time.wday : 0U],
           (unsigned)rtc_time.hour, (unsigned)rtc_time.min, (unsigned)rtc_time.sec,
           (temp < 0) ? "-" : "",
           (unsigned)(temp_abs / 100U), (unsigned)(temp_abs % 100U));
  USB_Print(buf);
}

/* ---------------------------------- BME280 ------------------------------- */
static uint8_t BME_Detect(void)
{
  bme_status = BME280_Init(&bme, &hi2c1, BME280_I2C_ADDR_LOW);
  if (bme_status == BME280_OK) {
    return 1;
  }
  bme_status = BME280_Init(&bme, &hi2c1, BME280_I2C_ADDR_HIGH);
  return (bme_status == BME280_OK) ? 1 : 0;
}

/* ---------------------------------- MP2733 (MUX) ------------------------- */
static void MP2733_SetCustomCurrent(I2C_HandleTypeDef *hi2c, uint8_t reg_addr, uint8_t mask, uint8_t value)
{
  uint8_t reg_val = 0;
  if (HAL_I2C_Mem_Read(hi2c, MP2733_ADDR << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, &reg_val, 1, 100) == HAL_OK) {
    reg_val &= ~mask;
    reg_val |= (value & mask);
    HAL_I2C_Mem_Write(hi2c, MP2733_ADDR << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, &reg_val, 1, 100);
  }
}

static uint8_t MP2733_Setup(MP2733_HandleTypeDef *h, const char *nome)
{
  char buf[96];
  uint8_t init_ok = MP2733_Init(h, &hi2c1);
  HAL_Delay(50);

  if (HAL_I2C_IsDeviceReady(&hi2c1, MP2733_ADDR << 1, 3, 100) != HAL_OK) {
    snprintf(buf, sizeof(buf), "%s -> Nao respondeu\r\n", nome);
    USB_Print(buf);
    return 0;
  }

  if (!MP2733_GetStatus(h)) {
    snprintf(buf, sizeof(buf), "%s -> Erro na comunicacao\r\n", nome);
    USB_Print(buf);
    return 0;
  }

  if (MP2733_WriteConfig(h)) {
    MP2733_SetCustomCurrent(&hi2c1, MP2733_REG_IINLIM, MP2733_IINLIM_MASK, MP2733_IINLIM_1_5A);
    MP2733_SetCustomCurrent(&hi2c1, MP2733_REG_ICHG, MP2733_ICHG_MASK, MP2733_ICHG_1_0A);
    snprintf(buf, sizeof(buf), "%s -> Configurado OK (Iout = 1.0 A)\r\n", nome);
    USB_Print(buf);
    return init_ok ? 1 : 0;
  }

  snprintf(buf, sizeof(buf), "%s -> Erro na configuracao\r\n", nome);
  USB_Print(buf);
  return 0;
}

static void MP2733_ReadAndPrint(MP2733_HandleTypeDef *h, const char *nome, uint8_t *status_out, uint8_t *fault_out)
{
  char buf[112];
  uint8_t reg00 = 0, reg05 = 0;

  /* Reaplica limites para evitar reset pelo watchdog do MP2733 */
  MP2733_SetCustomCurrent(&hi2c1, MP2733_REG_IINLIM, MP2733_IINLIM_MASK, MP2733_IINLIM_1_5A);
  MP2733_SetCustomCurrent(&hi2c1, MP2733_REG_ICHG, MP2733_ICHG_MASK, MP2733_ICHG_1_0A);

  HAL_I2C_Mem_Read(&hi2c1, MP2733_ADDR << 1, 0x00, I2C_MEMADD_SIZE_8BIT, &reg00, 1, 100);
  HAL_I2C_Mem_Read(&hi2c1, MP2733_ADDR << 1, 0x05, I2C_MEMADD_SIZE_8BIT, &reg05, 1, 100);

  uint16_t iin_lim_ma = 100U + ((reg00 & 0x3FU) * 50U);
  uint16_t ichg_ma    = (reg05 & 0x7FU) * 40U;

  snprintf(buf, sizeof(buf), "[%s] IinLim: %u mA (0x%02X) | Ichg: %u mA (0x%02X)\r\n",
           nome, iin_lim_ma, reg00, ichg_ma, reg05);
  USB_Print(buf);

  uint8_t st = MP2733_GetStatus(h);
  uint8_t fl = MP2733_GetFault(h);

  if (status_out != NULL) *status_out = st;
  if (fault_out  != NULL) *fault_out  = fl;
}

/* ---------------------------------- MAX17048 (MUX) ----------------------- */
static uint8_t MAX17048_Setup(MAX17048_HandleTypeDef *h, const char *nome)
{
  char buf[96];

  if (HAL_I2C_IsDeviceReady(&hi2c1, MAX17048_I2C_ADDR_8BIT, 3, 100) != HAL_OK) {
    snprintf(buf, sizeof(buf), "%s -> Nao respondeu (verifique bateria)\r\n", nome);
    USB_Print(buf);
    return 0;
  }

  HAL_Delay(10);
  if (MAX17048_Init(h, &hi2c1) != HAL_OK) {
    snprintf(buf, sizeof(buf), "%s -> Falha ao inicializar\r\n", nome);
    USB_Print(buf);
    return 0;
  }

  uint16_t version = 0;
  if (MAX17048_ReadVersion(h, &version) != HAL_OK) {
    snprintf(buf, sizeof(buf), "%s -> Falha ao ler VERSION (0x08)\r\n", nome);
    USB_Print(buf);
    return 0;
  }

  snprintf(buf, sizeof(buf), "%s -> Configurado OK (Ver: 0x%04X)\r\n", nome, version);
  USB_Print(buf);
  return 1;
}

static void MAX17048_ReadAndPrint(MAX17048_HandleTypeDef *h, const char *nome, uint16_t *vbat_mv_out, uint16_t *soc_out)
{
  char buf[96];
  float soc  = 0.0f;
  float vbat = 0.0f;

  HAL_StatusTypeDef err_soc  = MAX17048_GetSOC(h, &soc);
  HAL_StatusTypeDef err_vbat = MAX17048_GetVoltage(h, &vbat);

  if (err_soc == HAL_OK && err_vbat == HAL_OK) {
    int32_t soc_centi = (int32_t)(soc * 100.0f + 0.5f);
    int32_t vbat_mv   = (int32_t)(vbat * 1000.0f + 0.5f);

    if (vbat_mv_out != NULL && vbat_mv >= 0) {
      *vbat_mv_out = (uint16_t)vbat_mv; /* Tensao em mV */
    }
    if (soc_out != NULL && soc >= 0.0f) {
      *soc_out = (uint16_t)(soc + 0.5f);
    }

    snprintf(buf, sizeof(buf), "[%s] SOC: %ld.%02ld%% | Vbat: %ld.%03ld V\r\n",
             nome,
             (long)(soc_centi / 100), (long)(soc_centi % 100),
             (long)(vbat_mv / 1000),  (long)(vbat_mv % 1000));
    USB_Print(buf);
  } else {
    snprintf(buf, sizeof(buf), "[%s] Erro leitura I2C (SOC=%d, Vbat=%d)\r\n",
             nome, (int)err_soc, (int)err_vbat);
    USB_Print(buf);
  }
}

/* ---------------------------------- INA3221 (MUX) ------------------------ */
static uint8_t INA3221_SetupChannel(INA3221_Handle *h, uint8_t addr_7bit, const char *nome)
{
  char buf[96];

  if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr_7bit << 1), 2, 50) != HAL_OK) {
    snprintf(buf, sizeof(buf), "%s -> Nao respondeu\r\n", nome);
    USB_Print(buf);
    return 0;
  }

  INA3221_init(h, &hi2c1, (INA3221_Addr)addr_7bit, 200u);
  INA3221_set_cfg(h, INA3221_CFG_RESET);
  HAL_Delay(10);

  if (INA3221_default_config(h) == INA3221_OK) {
    snprintf(buf, sizeof(buf), "%s -> Configurado OK\r\n", nome);
    USB_Print(buf);
    return 1;
  }

  snprintf(buf, sizeof(buf), "%s -> Falha na configuracao\r\n", nome);
  USB_Print(buf);
  return 0;
}

/* INA1 (0x40 - MUX CH2): le tensao nos 3 canais */
static void INA1_ReadAndPrint(INA3221_Handle *h)
{
  static const INA3221_Chan chans[3] = { INA3221_CHAN_1, INA3221_CHAN_2, INA3221_CHAN_3 };
  float tensao[3] = { 0.0f, 0.0f, 0.0f };
  int32_t v_mv[3] = { 0, 0, 0 };
  uint32_t v_abs[3] = { 0, 0, 0 };
  char buf[112];

  for (uint8_t i = 0; i < 3U; i++) {
    if (INA3221_get_bus_voltage(h, chans[i], &tensao[i]) != INA3221_OK) {
      snprintf(buf, sizeof(buf), "INA1 (0x40) [CH%u] -> Erro de leitura I2C\r\n", (unsigned)(i + 1U));
      USB_Print(buf);
      return;
    }
    v_mv[i]  = (int32_t)(tensao[i] * 1000.0f + ((tensao[i] >= 0.0f) ? 0.5f : -0.5f));
    v_abs[i] = (uint32_t)((v_mv[i] < 0) ? -v_mv[i] : v_mv[i]);
  }

  /* Mapeamento para telemetria (ajuste conforme os barramentos fisicos da placa) */
  t1.tensao_13v8 = tensao[0];
  t1.tensao_5v   = tensao[1];

  snprintf(buf, sizeof(buf),
           "INA1 (0x40) -> V1: %s%lu.%03lu V | V2: %s%lu.%03lu V | V3: %s%lu.%03lu V\r\n",
           (v_mv[0] < 0) ? "-" : "", (unsigned long)(v_abs[0] / 1000U), (unsigned long)(v_abs[0] % 1000U),
           (v_mv[1] < 0) ? "-" : "", (unsigned long)(v_abs[1] / 1000U), (unsigned long)(v_abs[1] % 1000U),
           (v_mv[2] < 0) ? "-" : "", (unsigned long)(v_abs[2] / 1000U), (unsigned long)(v_abs[2] % 1000U));
  USB_Print(buf);
}

/* INA2 (0x41 - MUX CH3): le corrente nos canais 1 e 2, e tensao no canal 3 */
static void INA2_ReadAndPrint(INA3221_Handle *h)
{
  float corrente1 = 0.0f;
  float corrente2 = 0.0f;
  float tensao3   = 0.0f;
  char buf[112];

  INA3221_Error err_i1 = INA3221_get_shunt_current(h, INA3221_CHAN_1, &corrente1);
  INA3221_Error err_i2 = INA3221_get_shunt_current(h, INA3221_CHAN_2, &corrente2);
  INA3221_Error err_v3 = INA3221_get_bus_voltage(h, INA3221_CHAN_3, &tensao3);

  if (err_i1 == INA3221_OK && err_i2 == INA3221_OK && err_v3 == INA3221_OK) {
    int32_t  i1_centi = (int32_t)(corrente1 * 100.0f + ((corrente1 >= 0.0f) ? 0.5f : -0.5f));
    uint32_t i1_abs   = (uint32_t)((i1_centi < 0) ? -i1_centi : i1_centi);

    int32_t  i2_centi = (int32_t)(corrente2 * 100.0f + ((corrente2 >= 0.0f) ? 0.5f : -0.5f));
    uint32_t i2_abs   = (uint32_t)((i2_centi < 0) ? -i2_centi : i2_centi);

    int32_t  v3_mv    = (int32_t)(tensao3 * 1000.0f + ((tensao3 >= 0.0f) ? 0.5f : -0.5f));
    uint32_t v3_abs   = (uint32_t)((v3_mv < 0) ? -v3_mv : v3_mv);

    /* Mapeamento para telemetria (corrente em A caso corrente1/2 esteja em mA) */
    t1.corrente_13v8 = corrente1 / 1000.0f;
    t1.corrente_5v   = corrente2 / 1000.0f;

    snprintf(buf, sizeof(buf),
             "INA2 (0x41) -> I1: %s%lu.%02lu mA | I2: %s%lu.%02lu mA | V3: %s%lu.%03lu V\r\n",
             (i1_centi < 0) ? "-" : "", (unsigned long)(i1_abs / 100U), (unsigned long)(i1_abs % 100U),
             (i2_centi < 0) ? "-" : "", (unsigned long)(i2_abs / 100U), (unsigned long)(i2_abs % 100U),
             (v3_mv < 0)    ? "-" : "", (unsigned long)(v3_abs / 1000U), (unsigned long)(v3_abs % 1000U));
    USB_Print(buf);
  } else {
    USB_Print("INA2 (0x41) -> Erro de leitura I2C\r\n");
  }
}

/* ---------------------------------- Telemetria --------------------------- */
static void Telemetry_InitDefaults(void)
{
  memset(&t1, 0, sizeof(t1));

  t1.hora    = 9;
  t1.minuto  = 57;
  t1.segundo = 14;
  t1.dia     = 18;
  t1.mes     = 5;
  t1.ano     = 26;

  t1.temperatura_bateria = 21.5763f;
  t1.temperatura_externa = -19.3462f;
  t1.deploy_antena       = 15;

  t1.tensao_13v8   = 12.5342f;
  t1.tensao_5v     = 4.987f;
  t1.corrente_13v8 = 1.320f;
  t1.corrente_5v   = 0.874f;

  t1.tensao_painel_solar_1 = 5.120f;
  t1.tensao_painel_solar_2 = 4.980f;
  t1.tensao_painel_solar_3 = 5.054f;
  t1.tensao_painel_solar_4 = 4.763f;

  t1.status_carregador_1 = 0;
  t1.status_carregador_2 = 0;
  t1.falha_carregador_1  = 0;
  t1.falha_carregador_2  = 0;

  t1.adc_tensao_sistema_1   = 210;
  t1.adc_tensao_sistema_2   = 208;
  t1.adc_tensao_ntc_1       = 97;
  t1.adc_tensao_ntc_2       = 102;
  t1.adc_corrente_carga_1   = 134;
  t1.adc_corrente_carga_2   = 129;
  t1.adc_corrente_sistema_1 = 88;
  t1.adc_corrente_sistema_2 = 91;

  t1.tensao_bateria_1              = 0;
  t1.tensao_bateria_2              = 0;
  t1.carga_bateria_1               = 0;
  t1.carga_bateria_2               = 0;
  t1.taxa_carga_descarga_bateria_1 = 150;
  t1.taxa_carga_descarga_bateria_2 = -320;
}

/* ---------------------------------- UART Callbacks ----------------------- */
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (Size >= RX_BUF_SIZE) {
    Size = RX_BUF_SIZE - 1;
  }

  if (huart->Instance == USART1) {
    memcpy(msg_u1, rx_u1, Size);
    msg_u1[Size] = '\0';
    new_data_u1 = 1;
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, rx_u1, RX_BUF_SIZE - 1);
  } else if (huart->Instance == USART2) {
    memcpy(msg_u2, rx_u2, Size);
    msg_u2[Size] = '\0';
    new_data_u2 = 1;
    HAL_UARTEx_ReceiveToIdle_IT(&huart2, rx_u2, RX_BUF_SIZE - 1);
  } else if (huart->Instance == USART3) {
    memcpy(msg_u3, rx_u3, Size);
    msg_u3[Size] = '\0';
    new_data_u3 = 1;
    HAL_UARTEx_ReceiveToIdle_IT(&huart3, rx_u3, RX_BUF_SIZE - 1);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  __HAL_UART_CLEAR_OREFLAG(huart);
  __HAL_UART_CLEAR_FEFLAG(huart);
  __HAL_UART_CLEAR_NEFLAG(huart);

  if (huart->Instance == USART1) {
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, rx_u1, RX_BUF_SIZE - 1);
  } else if (huart->Instance == USART2) {
    HAL_UARTEx_ReceiveToIdle_IT(&huart2, rx_u2, RX_BUF_SIZE - 1);
  } else if (huart->Instance == USART3) {
    HAL_UARTEx_ReceiveToIdle_IT(&huart3, rx_u3, RX_BUF_SIZE - 1);
  }
}
/* USER CODE END 4 */

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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
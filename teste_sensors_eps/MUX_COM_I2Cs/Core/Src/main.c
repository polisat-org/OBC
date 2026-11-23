/* USER CODE BEGIN Header */
/**
  ******************************************************************************
- @file           : main.c
- @brief          : Main program body
  ******************************************************************************
- @attention
  *
- Copyright (c) 2026 STMicroelectronics.
- All rights reserved.
  *
- This software is licensed under terms that can be found in the LICENSE file
- in the root directory of this software component.
- If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ina3221.h"
#include "mp2733.h"
#include "max17048.h"

#define MUX_ADDR  (0x70)
#define INA_ADDR  (0x40)
#define INA_ADDR2 (0x41)
// MP2733_ADDR (0x4B) definido em mp2733.h
// MAX17048_I2C_ADDR_8BIT (0x6C -> 7-bit 0x36) definido em max17048.h

// Canais mapeados no MUX
#define CH_BAT1   0   // MP1 (0x4B) + MAX1 (0x36)
#define CH_BAT2   1   // MP2 (0x4B) + MAX2 (0x36)
#define CH_MP1    CH_BAT1
#define CH_MAX1   CH_BAT1
#define CH_MP2    CH_BAT2
#define CH_MAX2   CH_BAT2
#define CH_INA1   2   // INA1 (0x40)
#define CH_INA2   3   // INA2 (0x41)

// Configuracao de corrente de carga/saida do MP2733 (REG05h - ICC[6:0])
// Passo = 40 mA/bit -> 25 * 40 mA = 1000 mA (1.0 A)
#define MP2733_REG_ICHG        (0x05)
#define MP2733_ICHG_MASK       (0x7F)
#define MP2733_ICHG_1_0A       (0x19)

// Limite de corrente de entrada (REG00h - IINLIM[5:0]) para suportar 1.0 A na saida
// 100 mA + (28 * 50 mA) = 1500 mA (1.5 A), com EN_ILIM (bit 6) = 0
#define MP2733_REG_IINLIM      (0x00)
#define MP2733_IINLIM_MASK     (0x7F)
#define MP2733_IINLIM_1_5A     (0x1C)
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifndef LED_VERDE_GPIO_Port
#define LED_VERDE_GPIO_Port    GPIOB
#endif

#ifndef LED_VERDE_Pin
#define LED_VERDE_Pin          GPIO_PIN_5
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
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
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Scan do I2C, testa todos os enderecos nesse canal I2C
uint8_t scanI2C(I2C_HandleTypeDef *hi2c) {
    uint8_t qtde = 0;
    printf("Scan I2C...\r\n");
    for (uint8_t a = 1; a < 128; a++) {
        if (HAL_I2C_IsDeviceReady(hi2c, a << 1, 2, 20) == HAL_OK) {
            printf("Found 0x%02X\r\n", a);
            qtde++;
            HAL_Delay(100);
        }
        else {
            printf("Nothing in 0x%02X\r\n", a);
            HAL_Delay(10);
        }
    }
    return qtde;
}

// Manda byte para ativar canal do MUX
HAL_StatusTypeDef muxWrite(I2C_HandleTypeDef *hi2c, uint8_t data) {
    HAL_StatusTypeDef r = HAL_I2C_Master_Transmit(hi2c, MUX_ADDR << 1, &data, 1, 1000);
    if (r != HAL_OK)
        printf("Failed to write 0x%02X on MUX (err=0x%lX)\r\n",
               data, (unsigned long)HAL_I2C_GetError(hi2c));
    return r;
}

HAL_StatusTypeDef muxRead(I2C_HandleTypeDef *hi2c, uint8_t *out) {
    HAL_StatusTypeDef r = HAL_I2C_Master_Receive(hi2c, MUX_ADDR << 1, out, 1, 1000);
    if (r != HAL_OK)
        printf("Failed to read control byte on MUX (err=0x%lX)\r\n",
               (unsigned long)HAL_I2C_GetError(hi2c));
    return r;
}

HAL_StatusTypeDef selectChannel(I2C_HandleTypeDef *hi2c, uint8_t channel) {
    if (channel > 7) {
        printf("Invalid channel %d\r\n", channel);
        return HAL_ERROR;
    }
    return muxWrite(hi2c, (uint8_t)(1u << channel));
}

void scanMUX(I2C_HandleTypeDef *hi2c) {
    uint8_t data = 0;
    for (uint8_t channel = 0; channel < 8; channel++) {
        uint8_t qtde = 0;
        data = (1 << channel);
        if (HAL_I2C_Master_Transmit(hi2c, MUX_ADDR << 1, &data, 1, 1000) != HAL_OK) {
            printf("Failed to select channel %d (err=0x%lX)\r\n", channel, (unsigned long)HAL_I2C_GetError(hi2c));
        } else {
            printf("Channel %d selected on MUX.\r\n", channel);
            qtde = scanI2C(hi2c);
        }
        printf("End of scan channel %d, %d addresses\r\n", channel, qtde);
        HAL_Delay(100);
    }
    data = 0;
    HAL_I2C_Master_Transmit(hi2c, MUX_ADDR << 1, &data, 1, 1000);
}

uint8_t muxAddrTest(I2C_HandleTypeDef *hi2c) {
    for (uint8_t i = 0; i < 8; i++) {
        if (HAL_I2C_IsDeviceReady(hi2c, (MUX_ADDR + i) << 1, 5, 100) == HAL_OK)
            return MUX_ADDR + i;
    }
    return 0;
}

// --- Funcoes MP2733 ---

void MP2733_SetCustomCurrent(I2C_HandleTypeDef *hi2c, uint8_t reg_addr, uint8_t mask, uint8_t value)
{
    uint8_t reg_val = 0;
    if (HAL_I2C_Mem_Read(hi2c, MP2733_ADDR << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, &reg_val, 1, 100) == HAL_OK) {
        reg_val &= ~mask;
        reg_val |= (value & mask);
        HAL_I2C_Mem_Write(hi2c, MP2733_ADDR << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, &reg_val, 1, 100);
    }
}

static uint8_t inicializarMP2733(MP2733_HandleTypeDef *h, const char *nome)
{
    uint8_t init_ok = MP2733_Init(h, &hi2c1);
    HAL_Delay(100);

    if (HAL_I2C_IsDeviceReady(&hi2c1, MP2733_ADDR << 1, 3, 100) != HAL_OK) {
        printf("%s -> Nao esta pronto para comunicacao!\r\n", nome);
        return 0;
    }
    printf("%s -> Pronto para comunicacao!\r\n", nome);

    uint8_t comm_ok = MP2733_GetStatus(h);
    HAL_Delay(100);
    if (!comm_ok) {
        printf("%s -> Erro na comunicacao!\r\n", nome);
        return 0;
    }
    printf("%s -> Comunicacao OK!\r\n", nome);

    if (MP2733_WriteConfig(h)) {
        // Libera entrada ate 1.5 A para nao limitar a entrega de 1.0 A na saida
        MP2733_SetCustomCurrent(&hi2c1, MP2733_REG_IINLIM, MP2733_IINLIM_MASK, MP2733_IINLIM_1_5A);

        // Configura corrente de carga/saida (REG05h) para 1.0 A (0x19 = 25 * 40 mA)
        MP2733_SetCustomCurrent(&hi2c1, MP2733_REG_ICHG, MP2733_ICHG_MASK, MP2733_ICHG_1_0A);

        printf("%s -> Configurado com sucesso (Iout = 1.0 A)!\r\n", nome);
        return init_ok ? 1 : 0;
    } else {
        printf("%s -> Erro na configuracao!\r\n", nome);
        return 0;
    }
}

static void lerMP2733(MP2733_HandleTypeDef *h, const char *nome)
{
    // Reaplica os limites a cada ciclo para evitar reset pelo watchdog do MP2733
    MP2733_SetCustomCurrent(&hi2c1, MP2733_REG_IINLIM, MP2733_IINLIM_MASK, MP2733_IINLIM_1_5A);
    MP2733_SetCustomCurrent(&hi2c1, MP2733_REG_ICHG, MP2733_ICHG_MASK, MP2733_ICHG_1_0A);

    // Leitura de verificacao dos registradores configurados
    uint8_t reg00 = 0, reg05 = 0;
    HAL_I2C_Mem_Read(&hi2c1, MP2733_ADDR << 1, 0x00, I2C_MEMADD_SIZE_8BIT, &reg00, 1, 100);
    HAL_I2C_Mem_Read(&hi2c1, MP2733_ADDR << 1, 0x05, I2C_MEMADD_SIZE_8BIT, &reg05, 1, 100);

    uint16_t iin_lim_ma = 100 + ((reg00 & 0x3F) * 50);
    uint16_t ichg_ma    = (reg05 & 0x7F) * 40;

    printf("[%s] Limite Entrada: %u mA (0x%02X) | Limite Carga: %u mA (0x%02X)\r\n",
           nome, iin_lim_ma, reg00, ichg_ma, reg05);

    if (MP2733_GetStatus(h)) {
        MP2733_PrintStatus(h);
    }
    if (MP2733_GetFault(h)) {
        MP2733_PrintFault(h);
    }
}

// --- Funcoes MAX17048 ---

static uint8_t inicializarMAX17048(MAX17048_HandleTypeDef *h, const char *nome)
{
    if (HAL_I2C_IsDeviceReady(&hi2c1, MAX17048_I2C_ADDR_8BIT, 3, 100) != HAL_OK) {
        printf("%s -> Nao respondeu (verifique conexao/bateria)\r\n", nome);
        return 0;
    }

    HAL_Delay(10);
    if (MAX17048_Init(h, &hi2c1) != HAL_OK) {
        printf("%s -> Falha ao inicializar handle MAX17048\r\n", nome);
        return 0;
    }

    uint16_t version = 0;
    if (MAX17048_ReadVersion(h, &version) != HAL_OK) {
        printf("%s -> Falha ao ler registrador VERSION (0x08)\r\n", nome);
        return 0;
    }

    printf("%s -> Configurado OK (Version: 0x%04X)\r\n", nome, version);
    return 1;
}

static void lerMAX17048(MAX17048_HandleTypeDef *h, const char *nome)
{
    float soc  = 0.0f;
    float vbat = 0.0f;

    HAL_StatusTypeDef err_soc  = MAX17048_GetSOC(h, &soc);
    HAL_StatusTypeDef err_vbat = MAX17048_GetVoltage(h, &vbat);

    if (err_soc == HAL_OK && err_vbat == HAL_OK) {
        printf("[%s] SOC: %.2f%% | Vbat: %.4f V\r\n", nome, soc, vbat);
    } else {
        printf("[%s] Erro de leitura I2C (SOC=%d, Vbat=%d)\r\n", nome, err_soc, err_vbat);
    }
}
// --- Funcoes INA3221 ---

// INA1 (0x40 - MUX CH2): le tensao nos 3 canais
static void lerINA1(INA3221_Handle *h) {
    INA3221_Chan canais[3] = { INA3221_CHAN_1, INA3221_CHAN_2, INA3221_CHAN_3 };
    float tensao[3] = { 0.0f, 0.0f, 0.0f };

    for (uint8_t i = 0; i < 3; i++) {
        if (INA3221_get_bus_voltage(h, canais[i], &tensao[i]) != INA3221_OK) {
            printf("INA1 (0x40) [CH%d] -> Erro de leitura I2C\r\n", i + 1);
            return;
        }
    }

    printf("INA1 (0x40) -> V1: %.6f V | V2: %.6f V | V3: %.6f V\r\n",
           tensao[0], tensao[1], tensao[2]);
}

// INA2 (0x41 - MUX CH3): le corrente nos canais 1 e 2, e tensao no canal 3
static void lerINA2(INA3221_Handle *h) {
    float corrente1 = 0.0f;
    float corrente2 = 0.0f;
    float tensao3   = 0.0f;

    INA3221_Error err_i1 = INA3221_get_shunt_current(h, INA3221_CHAN_1, &corrente1);
    INA3221_Error err_i2 = INA3221_get_shunt_current(h, INA3221_CHAN_2, &corrente2);
    INA3221_Error err_v3 = INA3221_get_bus_voltage(h, INA3221_CHAN_3, &tensao3);

    if (err_i1 == INA3221_OK && err_i2 == INA3221_OK && err_v3 == INA3221_OK) {
        printf("INA2 (0x41) -> I1: %.6f mA | I2: %.6f mA | V3: %.6f V\r\n",
               corrente1, corrente2, tensao3);
    } else {
        printf("INA2 (0x41) -> Erro de leitura I2C\r\n");
    }
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  printf("=== INICIANDO SISTEMA ===\r\n");

  // 1. Inicializa Canal 0: MP 1 (0x4B) e MAX 1 (0x36)
  if (selectChannel(&hi2c1, CH_BAT1) == HAL_OK) {
      mp1_ativo  = inicializarMP2733(&mp1, "MP1 (CH0)");
      max1_ativo = inicializarMAX17048(&max1, "MAX1 (CH0)");
  }

  // 2. Inicializa Canal 1: MP 2 (0x4B) e MAX 2 (0x36)
  if (selectChannel(&hi2c1, CH_BAT2) == HAL_OK) {
      mp2_ativo  = inicializarMP2733(&mp2, "MP2 (CH1)");
      max2_ativo = inicializarMAX17048(&max2, "MAX2 (CH1)");
  }

  // 3. Inicializa INA 2 (0x41 - Canal 3)
  if (selectChannel(&hi2c1, CH_INA2) == HAL_OK) {
      if (HAL_I2C_IsDeviceReady(&hi2c1, INA_ADDR2 << 1, 2, 50) == HAL_OK) {
          INA3221_init(&ina2, &hi2c1, (INA3221_Addr)INA_ADDR2, 200u);
          INA3221_set_cfg(&ina2, INA3221_CFG_RESET);
          HAL_Delay(10);
          if (INA3221_default_config(&ina2) == INA3221_OK) {
              ina2_ativo = 1;
              printf("INA2 (CH3 - 0x41): Configurado OK\r\n");
          } else {
              printf("INA2 (CH3 - 0x41): Falha na configuracao\r\n");
          }
      } else {
          printf("INA2 (CH3 - 0x41): Nao respondeu\r\n");
      }
  }

  // 4. Inicializa INA 1 (0x40 - Canal 2)
  if (selectChannel(&hi2c1, CH_INA1) == HAL_OK) {
      if (HAL_I2C_IsDeviceReady(&hi2c1, INA_ADDR << 1, 2, 50) == HAL_OK) {
          INA3221_init(&ina1, &hi2c1, (INA3221_Addr)INA_ADDR, 200u);
          INA3221_set_cfg(&ina1, INA3221_CFG_RESET);
          HAL_Delay(10);
          if (INA3221_default_config(&ina1) == INA3221_OK) {
              ina1_ativo = 1;
              printf("INA1 (CH2 - 0x40): Configurado OK\r\n");
          } else {
              printf("INA1 (CH2 - 0x40): Falha na configuracao\r\n");
          }
      } else {
          printf("INA1 (CH2 - 0x40): Nao respondeu (verifique alimentacao)\r\n");
      }
  }

  // Isola todos os canais do MUX ao fim do boot
  muxWrite(&hi2c1, 0x00);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      HAL_GPIO_TogglePin(LED_VERDE_GPIO_Port, LED_VERDE_Pin);

      // 1. Leitura Canal 0: MP 1 (0x4B) e MAX 1 (0x36)
      if ((mp1_ativo || max1_ativo) && selectChannel(&hi2c1, CH_BAT1) == HAL_OK) {
          if (mp1_ativo) {
              lerMP2733(&mp1, "MP1 - CH0");
          }
          if (max1_ativo) {
              lerMAX17048(&max1, "MAX1 - CH0");
          }
      }

      // 2. Leitura Canal 1: MP 2 (0x4B) e MAX 2 (0x36)
      if ((mp2_ativo || max2_ativo) && selectChannel(&hi2c1, CH_BAT2) == HAL_OK) {
          if (mp2_ativo) {
              lerMP2733(&mp2, "MP2 - CH1");
          }
          if (max2_ativo) {
              lerMAX17048(&max2, "MAX2 - CH1");
          }
      }

      // 3. Leitura INA 2 (0x41 - Canal 3): I1, I2, V3
      if (ina2_ativo && selectChannel(&hi2c1, CH_INA2) == HAL_OK) {
          lerINA2(&ina2);
      }

      // 4. Leitura INA 1 (0x40 - Canal 2): V1, V2, V3
      if (ina1_ativo && selectChannel(&hi2c1, CH_INA1) == HAL_OK) {
          lerINA1(&ina1);
      }

      // Fecha todas as chaves do MUX entre amostras
      muxWrite(&hi2c1, 0x00);

      printf("----------------------------\r\n");
      HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
- @brief System Clock Configuration
- @retval None
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
- in the RCC_OscInitStruct structure.
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
- @brief I2C1 Initialization Function
- @param None
- @retval None
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
- @brief USART2 Initialization Function
- @param None
- @retval None
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
- @brief GPIO Initialization Function
- @param None
- @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_VERDE_GPIO_Port, LED_VERDE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_VERDE_Pin */
  GPIO_InitStruct.Pin = LED_VERDE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_VERDE_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int __io_putchar(int ch) {
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
/* USER CODE END 4 */

/**
- @brief  This function is executed in case of error occurrence.
- @retval None
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
- @brief  Reports the name of the source file and the source line number
-         where the assert_param error has occurred.
- @param  file: pointer to the source file name
- @param  line: assert_param error line source number
- @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
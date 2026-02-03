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
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Endereço I2C do MP2733
#define MP2733_ADDR 0x6C

// Registradores do MP2733
typedef enum {
    MP2733_REG_INPUT_CURRENT_LIMIT = 0x00,
    MP2733_REG_INPUT_VOLTAGE_REG = 0x01,
    MP2733_REG_TEMP_PROTECTION = 0x02,
    MP2733_REG_ADC_OTG_CTRL = 0x03,
    MP2733_REG_CHARGE_VSYS_CTRL = 0x04,
    MP2733_REG_CHARGE_CURRENT = 0x05,
    MP2733_REG_PRECHARGE_TERMINATION = 0x06,
    MP2733_REG_CHARGE_VOLTAGE = 0x07,
    MP2733_REG_TIMER_CONFIG = 0x08,
    MP2733_REG_BATTFET_CONFIG = 0x0A,
    MP2733_REG_INT_MASK_USB = 0x0B,
    MP2733_REG_STATUS = 0x0C,
    MP2733_REG_FAULT = 0x0D,
    MP2733_REG_ADC_BATTERY_VOLTAGE = 0x0E,
    MP2733_REG_ADC_SYS_VOLTAGE = 0x0F,
    MP2733_REG_ADC_THERMISTOR = 0x10,
    MP2733_REG_ADC_INPUT_VOLTAGE = 0x11,
    MP2733_REG_ADC_CHARGE_CURRENT = 0x12,
    MP2733_REG_ADC_INPUT_CURRENT = 0x13,
    MP2733_REG_POWER_STATUS = 0x14,
    MP2733_REG_DPM_MASK = 0x15,
    MP2733_REG_JEITA_CONFIG = 0x16,
    MP2733_REG_SAFETY_TIMER_STATUS = 0x17
} MP2733_Registers_t;

// Estrutura para status do carregador
typedef struct {
    uint8_t vsys_status;
    uint8_t thermal_reg_status;
    uint8_t ntc_float_status;
    uint8_t charge_status;
    uint8_t input_source;
} MP2733_Status_t;

// Estrutura para falhas
typedef struct {
    uint8_t ntc_fault;
    uint8_t battery_ov;
    uint8_t thermal_shutdown;
    uint8_t input_fault;
    uint8_t otg_fault;
    uint8_t watchdog_fault;
} MP2733_Fault_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MP2733_I2C_TIMEOUT 100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
uint8_t mp2733_initialized = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
// Funções para o MP2733
uint8_t MP2733_WriteRegister(uint8_t reg, uint8_t value);
uint8_t MP2733_ReadRegister(uint8_t reg, uint8_t *value);
uint8_t MP2733_Init(void);
uint8_t MP2733_GetStatus(MP2733_Status_t *status);
uint8_t MP2733_GetFault(MP2733_Fault_t *fault);
uint8_t MP2733_ReadADC(uint8_t adc_reg, uint16_t *adc_value);
void MP2733_PrintStatus(MP2733_Status_t status);
void MP2733_PrintFault(MP2733_Fault_t fault);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Função para escrever em um registrador do MP2733
uint8_t MP2733_WriteRegister(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};

    if (HAL_I2C_Master_Transmit(&hi2c1, MP2733_ADDR << 1, data, 2, MP2733_I2C_TIMEOUT) != HAL_OK)
    {
        return 0;
    }
    return 1;
}

// Função para ler um registrador do MP2733
uint8_t MP2733_ReadRegister(uint8_t reg, uint8_t *value)
{
    if (HAL_I2C_Master_Transmit(&hi2c1, MP2733_ADDR << 1, &reg, 1, MP2733_I2C_TIMEOUT) != HAL_OK)
    {
        return 0;
    }

    if (HAL_I2C_Master_Receive(&hi2c1, MP2733_ADDR << 1, value, 1, MP2733_I2C_TIMEOUT) != HAL_OK)
    {
        return 0;
    }
    return 1;
}

// Inicialização do MP2733 com valores da planilha
uint8_t MP2733_Init(void)
{
    // Configuração dos registradores conforme planilha
    if (!MP2733_WriteRegister(MP2733_REG_INPUT_CURRENT_LIMIT, 0x48)) return 0;        // 0b01001000
    if (!MP2733_WriteRegister(MP2733_REG_INPUT_VOLTAGE_REG, 0x06)) return 0;          // 0b00000110
    if (!MP2733_WriteRegister(MP2733_REG_TEMP_PROTECTION, 0xD9)) return 0;            // 0b11011001
    if (!MP2733_WriteRegister(MP2733_REG_ADC_OTG_CTRL, 0x10)) return 0;               // 0b00010000
    if (!MP2733_WriteRegister(MP2733_REG_CHARGE_VSYS_CTRL, 0xDB)) return 0;           // 0b11011011
    if (!MP2733_WriteRegister(MP2733_REG_CHARGE_CURRENT, 0xA6)) return 0;             // 0b10100110
    if (!MP2733_WriteRegister(MP2733_REG_PRECHARGE_TERMINATION, 0x22)) return 0;      // 0b00100010
    if (!MP2733_WriteRegister(MP2733_REG_CHARGE_VOLTAGE, 0xA0)) return 0;             // 0b10100000
    if (!MP2733_WriteRegister(MP2733_REG_TIMER_CONFIG, 0x95)) return 0;               // 0b10010101
    if (!MP2733_WriteRegister(MP2733_REG_BATTFET_CONFIG, 0x58)) return 0;             // 0b01011000
    if (!MP2733_WriteRegister(MP2733_REG_INT_MASK_USB, 0xC0)) return 0;               // 0b11000000
    if (!MP2733_WriteRegister(MP2733_REG_DPM_MASK, 0xE0)) return 0;                   // 0b11100000 (assumindo R=1)
    if (!MP2733_WriteRegister(MP2733_REG_JEITA_CONFIG, 0xEE)) return 0;               // 0b11101110

    mp2733_initialized = 1;
    return 1;
}

// Ler status do MP2733
uint8_t MP2733_GetStatus(MP2733_Status_t *status)
{
    uint8_t reg_value;

    if (!MP2733_ReadRegister(MP2733_REG_STATUS, &reg_value))
        return 0;

    status->vsys_status = (reg_value >> 0) & 0x01;
    status->thermal_reg_status = (reg_value >> 1) & 0x01;
    status->ntc_float_status = (reg_value >> 2) & 0x01;
    status->charge_status = (reg_value >> 3) & 0x03;
    status->input_source = (reg_value >> 5) & 0x07;

    return 1;
}

// Ler falhas do MP2733
uint8_t MP2733_GetFault(MP2733_Fault_t *fault)
{
    uint8_t reg_value;

    if (!MP2733_ReadRegister(MP2733_REG_FAULT, &reg_value))
        return 0;

    fault->ntc_fault = (reg_value >> 0) & 0x07;
    fault->battery_ov = (reg_value >> 3) & 0x01;
    fault->thermal_shutdown = (reg_value >> 4) & 0x01;
    fault->input_fault = (reg_value >> 5) & 0x01;
    fault->otg_fault = (reg_value >> 6) & 0x01;
    fault->watchdog_fault = (reg_value >> 7) & 0x01;

    return 1;
}

// Ler valores ADC
uint8_t MP2733_ReadADC(uint8_t adc_reg, uint16_t *adc_value)
{
    uint8_t lsb, msb;

    // Primeiro habilita a leitura ADC
    if (!MP2733_WriteRegister(MP2733_REG_ADC_OTG_CTRL, 0x11)) // Bit 0 = 1 para ler ADC
        return 0;

    HAL_Delay(1); // Pequeno delay para conversão

    // Lê o registrador ADC especificado
    if (!MP2733_ReadRegister(adc_reg, &lsb))
        return 0;

    if (!MP2733_ReadRegister(adc_reg + 1, &msb)) // Próximo registrador contém MSB
        return 0;

    *adc_value = (msb << 8) | lsb;

    // Desabilita leitura ADC
    if (!MP2733_WriteRegister(MP2733_REG_ADC_OTG_CTRL, 0x10)) // Bit 0 = 0
        return 0;

    return 1;
}

// Função para imprimir status (para debug)
void MP2733_PrintStatus(MP2733_Status_t status)
{
    const char *charge_states[] = {"Não carregando", "Trickle Charge", "Constant Current", "Charge Done"};
    const char *input_sources[] = {"Sem input", "USB SDP", "USB CDP", "USB DCP", "Adaptador", "OTG", "Reservado", "Reservado"};

    printf("VSYS Status: %s\r\n", status.vsys_status ? "Normal" : "Regulando");
    printf("Regulação Térmica: %s\r\n", status.thermal_reg_status ? "Ativa" : "Inativa");
    printf("NTC Float: %s\r\n", status.ntc_float_status ? "Ativo" : "Inativo");
    printf("Status Carga: %s\r\n", charge_states[status.charge_status]);
    printf("Fonte Input: %s\r\n", input_sources[status.input_source]);
}

// Função para imprimir falhas (para debug)
void MP2733_PrintFault(MP2733_Fault_t fault)
{
    const char *ntc_faults[] = {"Normal", "Reservado", "Warm", "Cool", "Reservado", "Cold", "Hot", "Reservado"};

    printf("Falha NTC: %s\r\n", ntc_faults[fault.ntc_fault]);
    printf("Overvoltage Bateria: %s\r\n", fault.battery_ov ? "Sim" : "Não");
    printf("Shutdown Térmico: %s\r\n", fault.thermal_shutdown ? "Sim" : "Não");
    printf("Falha Input: %s\r\n", fault.input_fault ? "Sim" : "Não");
    printf("Falha OTG: %s\r\n", fault.otg_fault ? "Sim" : "Não");
    printf("Watchdog: %s\r\n", fault.watchdog_fault ? "Expirado" : "OK");
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  // Inicializa o MP2733
  if (MP2733_Init())
  {
      printf("MP2733 inicializado com sucesso!\r\n");
  }
  else
  {
      printf("Erro na inicialização do MP2733!\r\n");
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
      MP2733_Status_t status;
      MP2733_Fault_t fault;
      uint16_t adc_value;

      if (mp2733_initialized)
      {
          // Lê e exibe status
          if (MP2733_GetStatus(&status))
          {
              MP2733_PrintStatus(status);
          }

          // Lê e exibe falhas
          if (MP2733_GetFault(&fault))
          {
              MP2733_PrintFault(fault);
          }

          // Lê tensão da bateria (exemplo)
          if (MP2733_ReadADC(MP2733_REG_ADC_BATTERY_VOLTAGE, &adc_value))
          {
              printf("ADC Bateria: %d\r\n", adc_value);
          }

          printf("---\r\n");
      }

      HAL_Delay(5000); // Espera 5 segundos
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Implementação da função printf para debug (se necessário)
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
/* USER CODE END 4 */

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

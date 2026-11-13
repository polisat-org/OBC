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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ina3221.h"
#include "mp2733.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SUCCESS_STR "+1000 aura\r\n"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  // Creates handles
     INA3221_Handle h;
     MP2733_HandleTypeDef MP2733;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // Initialization and reset

    // INA3221
      // this addr val is lowkey hardcoded
      INA3221_init(&h, &hi2c1, INA3221_ADDR_VS, 500u);

      printf("trying to reset ina3221...\r\n");
      INA3221_Error err;
      err = INA3221_set_cfg(&h,
        INA3221_CFG_RESET
      );
      if (err != INA3221_OK) {
        printf("error init INA: %u\r\n", err);
        continue; // commented out to allow MP2733 initialization even if INA3221 fails
      } else {
        printf(SUCCESS_STR);
      }

    // MP2733
      MP2733_Init(&MP2733, &hi2c1);
      uint8_t mp2733_initialized = MP2733_WriteConfig(&MP2733);
      if (mp2733_initialized)
      {
          printf("MP2733 inicializado com sucesso!\r\n");
      }
      else
      {
          printf("Erro na inicialização do MP2733!\r\n");
      }

    HAL_Delay(100u);

    // Configuration

    // INA
      printf("trying to configure ina3221...\r\n");
      err = INA3221_set_cfg(&h,
        INA3221_CFG_CHAN_1
        | INA3221_CFG_CHAN_2
        | INA3221_CFG_CHAN_3
        | INA3221_CFG_AVG_1
        | INA3221_CFG_TIME_BUS_1100_US
        | INA3221_CFG_TIME_SHUNT_1100_US
        | INA3221_CFG_MODE_SHUNT_AND_BUS_CONTINUOUS
      );
      if (err != INA3221_OK) {
        printf("error: %u\r\n", err);
        // continue;
      } else {
        printf(SUCCESS_STR);
      }

    // MP2733
      // Configuration is already done in MP2733_Init, so no need to configure again here

    HAL_Delay(100u);

    // INA die id reading
    printf("trying to read die id...\r\n");
    uint16_t id;
    err = INA3221_read_reg(&h, INA3221_REG_DIE_ID, &id);
    if (err != INA3221_OK) {
      printf("error: %u\r\n", err);
      // continue;
    } else {
      printf(SUCCESS_STR);
      printf("die id is %x in hex\r\n", id);
    }

    HAL_Delay(100u);

    unsigned i = 0u;

    // Configs done, starts reading loop 
    while (1) {
      //each loop reads a different INA channel
      INA3221_Chan chan;
      switch (i) {
        case 0u:
          chan = INA3221_CHAN_1;
          break;
        case 1u:
          chan = INA3221_CHAN_2;
          break;
        case 2u:
          chan = INA3221_CHAN_3;
          break;
        default: // unreachable
          chan = INA3221_CHAN_1;
          break;
      }

      INA3221_Reg reg_bus;
      INA3221_Reg reg_shunt;
      switch (chan) {
        case INA3221_CHAN_1:
          reg_bus = INA3221_REG_CHAN_1_BUS;
          reg_shunt = INA3221_REG_CHAN_1_SHUNT;
          break;
        case INA3221_CHAN_2:
          reg_bus = INA3221_REG_CHAN_2_BUS;
          reg_shunt = INA3221_REG_CHAN_2_SHUNT;
          break;
        case INA3221_CHAN_3:
          reg_bus = INA3221_REG_CHAN_3_BUS;
          reg_shunt = INA3221_REG_CHAN_3_SHUNT;
          break;
        default: // unreachable
          reg_bus = INA3221_REG_CHAN_1_BUS;
          reg_shunt = INA3221_REG_CHAN_1_SHUNT;
          break;
      }

      // printf("trying to read shunt voltage from channel %u\r\n", i);
      uint16_t v;
      err = INA3221_read_reg(&h, reg_shunt, &v);
      if (err != INA3221_OK) {
        printf("error: %u\r\n", err);
        //continue;
      } else {
        printf(SUCCESS_STR);
      }

      printf("current %u is %.1f ma\r\n ", i, (float)(v >> 3u) * 40e-6 * 1e3 * 1e1);
      printf("raw current %u is %.1f ma\r\n ", i, (float)((int16_t)v >> 3u) * 40e-6 * 1e3 * 1e1);

      // printf("trying to read bus voltage from channel %u\r\n", i);
      err = INA3221_read_reg(&h, reg_bus, &v);
      if (err != INA3221_OK) {
        printf("error: %u\r\n", err);
        // continue;
      } else {
        printf(SUCCESS_STR);
      }
      printf("voltage %u is %f V\r\n", i, (float)(v >> 3u) * 8e-3);

      i = (i+1u)%3u;

      if (mp2733_initialized)
      {
          // Lê e exibe status
          if (MP2733_GetStatus(&MP2733))
          {
              MP2733_PrintStatus(&MP2733);
          }

          // Lê e exibe falhas
          if (MP2733_GetFault(&MP2733))
          {
              MP2733_PrintFault(&MP2733);
          }

          printf("---\r\n");
      }
      else {
        
          printf("MP2733 não inicializado, pulando leitura de status e falhas.\r\n");
      }

      HAL_Delay(1000u);
    }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hi2c1.Init.Timing = 0x10D19CE4;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int __io_putchar(int ch) {
  HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
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

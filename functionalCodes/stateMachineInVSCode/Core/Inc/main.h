/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
/* Bench test with a single serial port: also accept telecommands on USART2 RX
 * (ST-Link virtual COM port), feeding the same path as USART1. Set to 0 for
 * flight builds, and when a PA3 -> PA10 jumper is in place (every byte would
 * be received twice). */
#define TC_ON_USART2 0
/* ADCS (Arduino Nano, 9600 8N1). PCB do OBC: USART2 (PA2 TX -> D12 do Nano, PA3 RX <- D11),
 * como no Notion; nesse caso o debug pela USART2 sai do ar. Bancada com o Nucleo, onde a
 * USART2 é o ST-Link: compile com -DBANCADA_ADCS_UART4=1 e o ADCS vai na UART4
 * (A0 = PA0 TX -> D12, A1 = PA1 RX <- D11). */
#if defined(BANCADA_ADCS_UART4) && BANCADA_ADCS_UART4
#define ADCS_NA_USART2 0
#else
#define ADCS_NA_USART2 1
#endif
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

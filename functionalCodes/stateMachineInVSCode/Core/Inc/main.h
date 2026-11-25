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
/* Bancada com o PC no lugar do TT&C: aceita telecomandos também pela USART2 RX (porta COM do ST-Link),
 * digitados num monitor serial como um caractere hexadecimal '0'..'9', 'a'..'f' (= 0x0..0xF; outros bytes,
 * como CR/LF, são ignorados). Entram no mesmo caminho da USART1. Só com o ADCS fora da USART2
 * (-DBANCADA_ADCS_UART4=1). Voo: 0. Ligue com -DTC_ON_USART2=1 (build_flash.sh --monitor). */
#ifndef TC_ON_USART2
#define TC_ON_USART2 0
#endif
/* ADCS (Arduino Nano, 9600 8N1). PCB do OBC: USART2 (PA2 TX -> D12 do Nano, PA3 RX <- D11),
 * como no Notion; nesse caso o debug pela USART2 sai do ar. Bancada com o Nucleo, onde a
 * USART2 é o ST-Link: compile com -DBANCADA_ADCS_UART4=1 e o ADCS vai na UART4
 * (A0 = PA0 TX -> D12, A1 = PA1 RX <- D11). */
#if defined(BANCADA_ADCS_UART4) && BANCADA_ADCS_UART4
#define ADCS_NA_USART2 0
#else
#define ADCS_NA_USART2 1
#endif
#if TC_ON_USART2 && ADCS_NA_USART2
#error "TC_ON_USART2 precisa do ADCS fora da USART2 (-DBANCADA_ADCS_UART4=1)"
#endif
/* Período da telemetria 'T' (housekeeping). Voo: 60 s. */
#ifndef HK_PERIODO_MS
#define HK_PERIODO_MS 60000U
#endif
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

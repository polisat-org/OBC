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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
typedef struct __attribute__((packed)) {
    // OBC
    uint8_t hora;
    uint8_t minuto;
    uint8_t segundo;
    uint8_t dia;
    uint8_t mes;
    uint8_t ano;

    // // Mecanica
    float temperatura_bateria;
    // float temperatura_estrutura;
    // float umidade;
    // float pressao;
    // uint8_t deploy_antena;

    // // EPS
    // // Tensao e corrente 13.8V e 5V
    // float tensao_13v8;
    // float tensao_5v;
    // float corrente_13v8;
    // float corrente_5v;

    // // Paineis Solares
    // float tensao_painel_solar_1;
    // float tensao_painel_solar_2;
    // float tensao_painel_solar_3;
    // float tensao_painel_solar_4;

    // // Status e Falhas dos Carregadores
    // uint8_t status_carregador_1;
    // uint8_t status_carregador_2;
    // uint8_t falha_carregador_1;
    // uint8_t falha_carregador_2;

    // // ADC
    // uint8_t adc_tensao_sistema_1;
    // uint8_t adc_tensao_sistema_2;
    // uint8_t adc_tensao_ntc_1;
    // uint8_t adc_tensao_ntc_2;
    // uint8_t adc_corrente_carga_1;
    // uint8_t adc_corrente_carga_2;
    // uint8_t adc_corrente_sistema_1;
    // uint8_t adc_corrente_sistema_2;

    // // Baterias
    // uint16_t tensao_bateria_1;
    // uint16_t tensao_bateria_2;
    // uint16_t carga_bateria_1;
    // uint16_t carga_bateria_2;
    // int16_t  taxa_carga_descarga_bateria_1;
    // int16_t  taxa_carga_descarga_bateria_2;
} Telemetry_t;

typedef enum {
	EVT_HK_DONE,
	EVT_RX_DONE,
	EVT_TX_DONE,
	EVT_MISSION_DONE,
	EVT_DEPLOY_DONE,
	EVT_DETUMBLING_DONE,
} EventType_t;

typedef struct {
	EventType_t type;
	uint32_t timestamp;
	void *content;
} Event_t;

typedef enum {
	STATE_IDLE,
	STATE_TRANSMIT,
	STATE_MISSION,
	STATE_DEPLOY,
	STATE_DETUMBLING,
} SatState_t;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

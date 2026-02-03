/**
 ******************************************************************************
 * @file           : obc_comm.h
 * @brief          : OBC Communication Library Header
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

#ifndef OBC_COMM_H
#define OBC_COMM_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>

/* Public defines ------------------------------------------------------------*/

// Pinos GPIO
#define TTC_BUSY_PIN GPIO_PIN_4
#define ADCS_ON_OFF_PIN GPIO_PIN_5
#define PL_START_PIN GPIO_PIN_6
#define PL_END_PIN GPIO_PIN_7

#define TTC_BUSY_PORT GPIOA
#define ADCS_ON_OFF_PORT GPIOA
#define PL_START_PORT GPIOA
#define PL_END_PORT GPIOA

// Identificadores da tabela do TT&C
#define ID_ADCS_00 "100000"
#define ID_ADCS_01 "100001"
#define ID_ADCS_10 "100010"

#define ID_PAYLOAD_00 "100100"
#define ID_PAYLOAD_01 "100101"

#define ID_EPS_00 "101000"

#define ID_MECANICA_00 "101100"
#define ID_MECANICA_01 "101101"

#define ID_OBC_00 "110000"

#define ID_TTC_00 "110100"
#define ID_TTC_01 "110101"
#define ID_TTC_10 "110110"

#define UART_TIMEOUT 1000

    /* Public typedefs -----------------------------------------------------------*/

    typedef enum
    {
        SUBSYS_ADCS = 0,
        SUBSYS_CARGA_UTIL,
        SUBSYS_EPS,
        SUBSYS_MECANICA,
        SUBSYS_OBC,
        SUBSYS_TTC
    } Subsystem_t;

    typedef struct
    {
        uint8_t area[2];
        uint8_t id[6];
    } Identifier_t;

    /* Public variables ----------------------------------------------------------*/
    // Declaração das variáveis globais (definidas no .c)
    extern volatile uint8_t adcs_ligado;
    extern volatile uint8_t carga_util_ativa;

    /* Public function prototypes ------------------------------------------------*/

    // Funções de inicialização
    void OBC_COMM_Init(UART_HandleTypeDef *huart_ttc, UART_HandleTypeDef *huart_adcs, UART_HandleTypeDef *huart_payload);

    // Funções para comunicação com subsistemas
    uint8_t TT_C_SendCommand(const uint8_t *command, uint16_t size);
    uint8_t TT_C_ReceiveData(uint8_t *buffer, uint16_t size);
    uint8_t ADCS_SendCommand(const uint8_t *command, uint16_t size);
    uint8_t ADCS_ReceiveData(uint8_t *buffer, uint16_t size);
    uint8_t CargaUtil_SendCommand(const uint8_t *command, uint16_t size);
    uint8_t CargaUtil_ReceiveData(uint8_t *buffer, uint16_t size);

    // Funções de controle dos subsistemas
    void ADCS_LigaDesliga(uint8_t estado);
    void CargaUtil_Start(void);
    void CargaUtil_End(void);
    uint8_t TT_C_IsBusy(void);

    // Funções auxiliares
    void BuildIdentifier(Subsystem_t subsys, uint8_t area_code, uint8_t *output);

    // Função de processamento principal (para ser chamada no loop principal)
    void OBC_COMM_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* OBC_COMM_H */
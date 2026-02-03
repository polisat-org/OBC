/**
 ******************************************************************************
 * @file           : obc_comm.c
 * @brief          : OBC Communication Library Implementation
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

/* Includes ------------------------------------------------------------------*/
#include "obc_comm.h"

/* Private variables ---------------------------------------------------------*/

// Handlers UART (definidos pelo usuário)
static UART_HandleTypeDef *huart_ttc = NULL;
static UART_HandleTypeDef *huart_adcs = NULL;
static UART_HandleTypeDef *huart_payload = NULL;

// Buffers para comunicação
static uint8_t rx_buffer_ttc[256];
static uint8_t tx_buffer_ttc[256];
static uint8_t rx_buffer_adcs[256];
static uint8_t tx_buffer_adcs[256];
static uint8_t rx_buffer_payload[256];
static uint8_t tx_buffer_payload[256];

// Estados dos subsistemas
volatile uint8_t adcs_ligado = 0;
volatile uint8_t carga_util_ativa = 0;

/* Private function prototypes -----------------------------------------------*/
static void StartUARTReception(void);

/* Public function implementations -------------------------------------------*/

/**
 * @brief  Inicializa a biblioteca de comunicação OBC
 * @param  huart_ttc_ptr: Handler UART para TT&C
 * @param  huart_adcs_ptr: Handler UART para ADCS
 * @param  huart_payload_ptr: Handler UART para Carga Útil
 * @retval None
 */
void OBC_COMM_Init(UART_HandleTypeDef *huart_ttc_ptr, UART_HandleTypeDef *huart_adcs_ptr, UART_HandleTypeDef *huart_payload_ptr)
{
    huart_ttc = huart_ttc_ptr;
    huart_adcs = huart_adcs_ptr;
    huart_payload = huart_payload_ptr;

    // Inicialização dos subsistemas
    ADCS_LigaDesliga(0); // Inicia com ADCS desligado
    CargaUtil_End();     // Inicia com carga útil desativada

    // Inicia recepção UART para todos os subsistemas
    StartUARTReception();
}

/**
 * @brief  Processamento principal da comunicação OBC (chamar no loop principal)
 * @retval None
 */
void OBC_COMM_Process(void)
{
    // Verificar se TT&C não está busy e enviar comando
    if (!TT_C_IsBusy())
    {
        uint8_t command[] = "COMANDO_TTC";
        TT_C_SendCommand(command, sizeof(command));
    }

    // Verificar se está ligado e enviar comando ADCS
    if (adcs_ligado)
    {
        uint8_t adcs_cmd[] = "COMANDO_ADCS";
        ADCS_SendCommand(adcs_cmd, sizeof(adcs_cmd));
    }

    // Verificar carga útil e enviar comando
    if (carga_util_ativa)
    {
        uint8_t carga_cmd[] = "COMANDO_CARGA";
        CargaUtil_SendCommand(carga_cmd, sizeof(carga_cmd));
    }
}

/* Communication functions implementation */

uint8_t TT_C_SendCommand(const uint8_t *command, uint16_t size)
{
    if (TT_C_IsBusy() || huart_ttc == NULL)
    {
        return 0; // Falha - subsistema ocupado ou não inicializado
    }

    HAL_StatusTypeDef status = HAL_UART_Transmit(huart_ttc, command, size, UART_TIMEOUT);
    return (status == HAL_OK);
}

uint8_t TT_C_ReceiveData(uint8_t *buffer, uint16_t size)
{
    if (TT_C_IsBusy() || huart_ttc == NULL)
    {
        return 0; // Falha - subsistema ocupado ou não inicializado
    }

    HAL_StatusTypeDef status = HAL_UART_Receive(huart_ttc, buffer, size, UART_TIMEOUT);
    return (status == HAL_OK);
}

uint8_t ADCS_SendCommand(const uint8_t *command, uint16_t size)
{
    if (!adcs_ligado || huart_adcs == NULL)
    {
        return 0; // Falha - ADCS desligado ou não inicializado
    }

    return (HAL_UART_Transmit(huart_adcs, command, size, UART_TIMEOUT) == HAL_OK);
}

uint8_t ADCS_ReceiveData(uint8_t *buffer, uint16_t size)
{
    if (!adcs_ligado || huart_adcs == NULL)
    {
        return 0; // Falha - ADCS desligado ou não inicializado
    }

    return (HAL_UART_Receive(huart_adcs, buffer, size, UART_TIMEOUT) == HAL_OK);
}

uint8_t CargaUtil_SendCommand(const uint8_t *command, uint16_t size)
{
    if (!carga_util_ativa || huart_payload == NULL)
    {
        return 0; // Falha - carga útil não ativa ou não inicializado
    }

    return (HAL_UART_Transmit(huart_payload, command, size, UART_TIMEOUT) == HAL_OK);
}

uint8_t CargaUtil_ReceiveData(uint8_t *buffer, uint16_t size)
{
    if (!carga_util_ativa || huart_payload == NULL)
    {
        return 0; // Falha - carga útil não ativa ou não inicializado
    }

    return (HAL_UART_Receive(huart_payload, buffer, size, UART_TIMEOUT) == HAL_OK);
}

/* Subsystem control functions implementation */

void ADCS_LigaDesliga(uint8_t estado)
{
    adcs_ligado = estado;
    HAL_GPIO_WritePin(ADCS_ON_OFF_PORT, ADCS_ON_OFF_PIN,
                      estado ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void CargaUtil_Start(void)
{
    carga_util_ativa = 1;
    HAL_GPIO_WritePin(PL_START_PORT, PL_START_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PL_END_PORT, PL_END_PIN, GPIO_PIN_RESET);
}

void CargaUtil_End(void)
{
    carga_util_ativa = 0;
    HAL_GPIO_WritePin(PL_START_PORT, PL_START_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PL_END_PORT, PL_END_PIN, GPIO_PIN_SET);
}

uint8_t TT_C_IsBusy(void)
{
    // Lê o estado atual do pino TTC_BUSY (agora configurado como entrada)
    return HAL_GPIO_ReadPin(TTC_BUSY_PORT, TTC_BUSY_PIN);
}

/* Utility functions implementation */

void BuildIdentifier(Subsystem_t subsys, uint8_t area_code, uint8_t *output)
{
    // Implementação para construir identificador conforme tabela TT&C
    // (a ser implementada conforme necessidade específica)
}

/* Private functions implementation */

/**
 * @brief  Inicia a recepção UART para todos os subsistemas
 * @retval None
 */
static void StartUARTReception(void)
{
    if (huart_payload != NULL)
    {
        HAL_UART_Receive_IT(huart_payload, rx_buffer_payload, 1);
    }
    if (huart_ttc != NULL)
    {
        HAL_UART_Receive_IT(huart_ttc, rx_buffer_ttc, 1);
    }
    if (huart_adcs != NULL)
    {
        HAL_UART_Receive_IT(huart_adcs, rx_buffer_adcs, 1);
    }
}

/* Callback functions */

/**
 * @brief  Callback de recepção UART (deve ser chamado do callback principal)
 * @param  huart: Handler UART que gerou a interrupção
 * @retval None
 */
void OBC_COMM_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart_payload != NULL && huart->Instance == huart_payload->Instance)
    {
        // Processa dados recebidos da carga útil
        HAL_UART_Receive_IT(huart_payload, rx_buffer_payload, 1);
    }
    else if (huart_ttc != NULL && huart->Instance == huart_ttc->Instance)
    {
        // Processa dados recebidos do TT&C
        HAL_UART_Receive_IT(huart_ttc, rx_buffer_ttc, 1);
    }
    else if (huart_adcs != NULL && huart->Instance == huart_adcs->Instance)
    {
        // Processa dados recebidos do ADCS
        HAL_UART_Receive_IT(huart_adcs, rx_buffer_adcs, 1);
    }
}
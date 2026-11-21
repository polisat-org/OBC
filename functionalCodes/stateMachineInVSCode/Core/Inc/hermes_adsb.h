/*
 * HERMES ADS-B -> OBC: receptor do stream de 4080 bytes por DMA.
 *
 * O protocolo (20 Plane_t de 204 bytes a cada 5 s, 115200 8N1) e fixo.
 * A UART/DMA usada pelo OBC NAO e fixa: veja hermes_config.h. Os padroes
 * reproduzem o stateMachineVSCode examinado, mas podem ser sobrescritos.
 *
 * Uma IRQ por byte a 115200 baud seriam 115200 interrupcoes/s -- inviavel
 * num OBC a 4 MHz. Por isso o transporte e feito por DMA e so uma IRQ por
 * bloco completo avisa a task (sem polling). O teste host em stm32_test/
 * modela a recepcao byte-a-byte por IT apenas para validar a logica da ISR.
 *
 * Por padrao nao mexe em USART1 (telecomandos), USART2 (debug/telemetria)
 * nem no layout de Telemetry_t (85 bytes).
 */
#ifndef HERMES_ADSB_H
#define HERMES_ADSB_H

#include "main.h"          /* traz stm32l4xx_hal.h */
#include "cmsis_os.h"

#include "hermes_config.h"   /* UART/DMA/tempos: trocar aqui, nao na logica */
#include "hermes_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    HERMES_ADSB_STATE_IDLE = 0U,
    HERMES_ADSB_STATE_READY,
    HERMES_ADSB_STATE_ERROR,
} HermesAdsbState_t;

typedef struct {
    volatile HermesAdsbState_t state;
    volatile uint32_t buffers;      /* blocos de 4080 bytes completos */
    volatile uint32_t overruns;     /* bloco chegou antes do Take() */
    volatile uint32_t dma_errors;
    volatile uint32_t resyncs;      /* DMA reiniciado por timeout */
    volatile uint32_t last_tick_ms; /* osKernelGetTickCount() do ultimo bloco */
} HermesAdsbStats_t;

/*
 * Chamado por HAL_UART_MspInit(), dentro do USER CODE do ramo da UART
 * escolhida. Configura o DMA de hermes_config.h e habilita as IRQs.
 */
void HermesAdsb_InitDma(UART_HandleTypeDef *huart);

/* Chamado pelo handler de IRQ do DMA (stm32l4xx_it.c). */
void HermesAdsb_DmaIrqHandler(void);

/*
 * Chamado no USER CODE 2 do main(), depois do MX_USARTx_UART_Init().
 * Arma o primeiro DMA de 4080 bytes.
 */
void HermesAdsb_Init(UART_HandleTypeDef *huart, osThreadId_t notifyTask);

/* Pode ser chamado depois que a task foi criada (USER CODE RTOS_THREADS ou
 * primeiro comando da propria task). */
void HermesAdsb_SetNotifyTask(osThreadId_t notifyTask);

/*
 * Chamados dentro de HAL_UART_RxCpltCallback / HAL_UART_ErrorCallback do
 * main.c, somente quando huart->Instance == HERMES_ADSB_UART_INSTANCE.
 */
void HermesAdsb_RxCpltCallback(UART_HandleTypeDef *huart);
void HermesAdsb_ErrorCallback(UART_HandleTypeDef *huart);

/*
 * Copia os 20 Plane_t do bloco pronto e libera o buffer para o DMA.
 * Devolve 1 se copiou, 0 se nao havia bloco pronto.
 * Chamado de dentro da task (nao da ISR).
 */
int HermesAdsb_Take(Plane_t out[HERMES_NUM_PLANES]);

/*
 * Chamar periodicamente na task. Se nenhum bloco chegou em
 * HERMES_ADSB_TIMEOUT_MS, aborta e reinicia o DMA -- isso realinha o
 * receptor caso o OBC tenha entrado no meio de um bloco (nao ha cabecalho).
 */
void HermesAdsb_CheckTimeout(uint32_t now_tick_ms);

const HermesAdsbStats_t *HermesAdsb_Stats(void);

#ifdef __cplusplus
}
#endif

#endif /* HERMES_ADSB_H */

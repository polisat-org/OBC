/*
 * Recepcao do stream HERMES no OBC por UART + DMA.
 * Sem header, sem CRC, sem polling: o bloco de 4080 bytes e a unidade.
 * UART/DMA/prioridades/timeout vem de hermes_config.h.
 */
#include "hermes_adsb.h"

#include <string.h>

static UART_HandleTypeDef *g_uart;
static osThreadId_t g_notify;
static DMA_HandleTypeDef g_hdma_rx;

/* Dois bancos de 4080 bytes: o DMA enche um enquanto a task le o outro. */
static uint8_t g_dma_buf[2][HERMES_BUFFER_SIZE] __attribute__((aligned(4)));
static volatile int8_t g_fill;   /* 0/1: banco que o DMA esta enchendo */
static volatile int8_t g_full;   /* -1 ou 0/1: banco pronto para a task */
static HermesAdsbStats_t g_stats;

void HermesAdsb_InitDma(UART_HandleTypeDef *huart)
{
    if (huart == NULL || huart->Instance != HERMES_ADSB_UART_INSTANCE) {
        return;
    }

    HERMES_ADSB_DMA_CLK_ENABLE();

    g_hdma_rx.Instance = HERMES_ADSB_DMA_INSTANCE;
    /* Em STM32L4 sem DMAMUX, Init.Request e o codigo CSEL (0..7) do canal,
     * nao um simbolo LL_DMAMUX_REQ_*. Os valores padrao (DMA1 Ch3 / req 2)
     * mapeiam USART3_RX no L476; troque tudo em hermes_config.h se o OBC
     * usar outro modelo. */
    g_hdma_rx.Init.Request = HERMES_ADSB_DMA_REQUEST;
    g_hdma_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    g_hdma_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    g_hdma_rx.Init.MemInc = DMA_MINC_ENABLE;
    g_hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    g_hdma_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    g_hdma_rx.Init.Mode = DMA_NORMAL;
    g_hdma_rx.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&g_hdma_rx) != HAL_OK) {
        HERMES_ADSB_ON_ERROR();
    }

    __HAL_LINKDMA(huart, hdmarx, g_hdma_rx);

    HAL_NVIC_SetPriority(HERMES_ADSB_DMA_IRQn, HERMES_ADSB_IRQ_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(HERMES_ADSB_DMA_IRQn);

    /* Necessario para HAL_UART_ErrorCallback avisar overrun/framing. */
    HAL_NVIC_SetPriority(HERMES_ADSB_UART_IRQn, HERMES_ADSB_IRQ_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(HERMES_ADSB_UART_IRQn);
}

void HermesAdsb_DmaIrqHandler(void)
{
    HAL_DMA_IRQHandler(&g_hdma_rx);
}

void HermesAdsb_SetNotifyTask(osThreadId_t notifyTask)
{
    g_notify = notifyTask;
}

static HAL_StatusTypeDef arm_dma(void)
{
    if (g_uart == NULL) {
        return HAL_ERROR;
    }
    return HAL_UART_Receive_DMA(g_uart, g_dma_buf[g_fill], HERMES_BUFFER_SIZE);
}

void HermesAdsb_Init(UART_HandleTypeDef *huart, osThreadId_t notifyTask)
{
    g_uart = huart;
    g_notify = notifyTask;
    g_fill = 0;
    g_full = -1;

    memset(&g_stats, 0, sizeof(g_stats));
    memset(g_dma_buf, 0, sizeof(g_dma_buf));
    g_stats.state = HERMES_ADSB_STATE_IDLE;
    g_stats.last_tick_ms = osKernelGetTickCount();

    __HAL_UART_CLEAR_FLAG(g_uart, UART_CLEAR_PEF | UART_CLEAR_FEF
            | UART_CLEAR_NEF | UART_CLEAR_OREF);

    if (arm_dma() != HAL_OK) {
        g_stats.state = HERMES_ADSB_STATE_ERROR;
    }
}

void HermesAdsb_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != g_uart) {
        return;                 /* nunca mexe com USART1/USART2 */
    }

    if (g_full >= 0) {
        /* A task ainda nao leu o bloco anterior: descarta este e
         * rearma no mesmo banco (o banco recem-chegado esta livre). */
        g_stats.overruns++;
    } else {
        g_full = g_fill;
        g_stats.buffers++;
        g_fill ^= 1;            /* proximo DMA vai para o outro banco */
    }

    g_stats.last_tick_ms = osKernelGetTickCount();
    g_stats.state = HERMES_ADSB_STATE_READY;

    (void)arm_dma();
    if (g_notify != NULL) {
        (void)osThreadFlagsSet(g_notify, HERMES_ADSB_FLAG);
    }
}

void HermesAdsb_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != g_uart) {
        return;
    }

    g_stats.dma_errors++;
    g_stats.last_tick_ms = osKernelGetTickCount();

    if (huart->RxState == HAL_UART_STATE_READY) {
        (void)arm_dma();
    }
}

int HermesAdsb_Take(Plane_t out[HERMES_NUM_PLANES])
{
    int8_t full;
    uint32_t primask;

    if (out == NULL) {
        return 0;
    }

    primask = __get_PRIMASK();
    __disable_irq();            /* 4080 bytes: curto, mas atomico */

    full = g_full;
    if (full < 0) {
        __set_PRIMASK(primask);
        return 0;
    }
    g_full = -1;                /* libera o banco para o proximo DMA */

    memcpy(out, g_dma_buf[full], HERMES_BUFFER_SIZE);
    __set_PRIMASK(primask);
    return 1;
}

void HermesAdsb_CheckTimeout(uint32_t now_tick_ms)
{
    if (g_uart == NULL) {
        return;
    }
    if ((now_tick_ms - g_stats.last_tick_ms) < HERMES_ADSB_TIMEOUT_MS) {
        return;
    }

    /* Sem cabecalho, a unica forma de re-sincronizar e recomeçar o DMA. */
    (void)HAL_UART_AbortReceive(g_uart);
    g_fill = 0;
    g_full = -1;
    memset(g_dma_buf, 0, sizeof(g_dma_buf));
    g_stats.resyncs++;
    g_stats.last_tick_ms = now_tick_ms;
    g_stats.state = HERMES_ADSB_STATE_IDLE;
    (void)arm_dma();
}

const HermesAdsbStats_t *HermesAdsb_Stats(void)
{
    return &g_stats;
}

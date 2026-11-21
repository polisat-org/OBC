/*
 * HERMES ADS-B no OBC - CONFIGURACAO
 *
 * Este e o UNICO lugar para trocar UART, DMA, prioridades e tempos do receptor.
 * Os valores abaixo sao apenas PADROES para o stateMachineVSCode examinado.
 * Se o OBC mudou (outra UART, outro DMA, outro clock), NAO edite a logica:
 * redefina os macros aqui ou passe -DHERMES_ADSB_... na linha do compilador.
 *
 * O protocolo em si (20 x 204 = 4080 bytes crus, 115200 8N1) NAO depende
 * de nenhum destes macros: eles escolhem somente ONDE o OBC recebe o stream.
 * A baud rate e os pinos fisicos sao configurados pelo CubeMX (.ioc) e nao
 * por este arquivo - confira MX_USARTx_UART_Init() e o GPIO da sua placa.
 */
#ifndef HERMES_CONFIG_H
#define HERMES_CONFIG_H

/* ---------------------------------------------------------------- UART --- */
/* Padrao: USART3, com PB11 = RX e PB10 = TX (pinos da PCB do OBC), handle gerado pelo CubeMX. */
#ifndef HERMES_ADSB_UART_INSTANCE
#define HERMES_ADSB_UART_INSTANCE        USART3
#endif
#ifndef HERMES_ADSB_UART_HANDLE
#define HERMES_ADSB_UART_HANDLE          huart3
#endif
#ifndef HERMES_ADSB_UART_IRQn
#define HERMES_ADSB_UART_IRQn            USART3_IRQn
#endif
#ifndef HERMES_ADSB_BAUD
#define HERMES_ADSB_BAUD                 115200U   /* so documentacao/conferencia */
#endif

/* ----------------------------------------------------------------- DMA --- */
/* STM32L4 sem DMAMUX: Init.Request e o codigo CSEL (0..7) do DMA_CSELR.
 * No L476, USART3_RX = DMA1 Channel3 / request 2 (RM0351 tabela 38).
 * Em outro modelo/clock, troque os quatro itens abaixo em conjunto. */
#ifndef HERMES_ADSB_DMA_CLK_ENABLE
#define HERMES_ADSB_DMA_CLK_ENABLE()     __HAL_RCC_DMA1_CLK_ENABLE()
#endif
#ifndef HERMES_ADSB_DMA_INSTANCE
#define HERMES_ADSB_DMA_INSTANCE         DMA1_Channel3
#endif
#ifndef HERMES_ADSB_DMA_REQUEST
#define HERMES_ADSB_DMA_REQUEST          DMA_REQUEST_2
#endif
#ifndef HERMES_ADSB_DMA_IRQn
#define HERMES_ADSB_DMA_IRQn             DMA1_Channel3_IRQn
#endif

/* Prioridade das IRQs do HERMES. Deve ser >= configLIBRARY_MAX_SYSCALL_... . */
#ifndef HERMES_ADSB_IRQ_PRIORITY
#define HERMES_ADSB_IRQ_PRIORITY         5U
#endif

/* --------------------------------------------------------------- RTOS ---- */
/* Bit de thread flag usado só pela task do HERMES (evite bits 0 e 1). */
#ifndef HERMES_ADSB_FLAG
#define HERMES_ADSB_FLAG                 (1U << 2)
#endif
/* Sem cabecalho no stream: se nada chegar nesse tempo, aborta/rearma o DMA. */
#ifndef HERMES_ADSB_TIMEOUT_MS
#define HERMES_ADSB_TIMEOUT_MS           7000U   /* > 5 s + margem */
#endif
/* A task acorda periodicamente para checar o timeout, mesmo sem chegada. */
#ifndef HERMES_ADSB_TASK_POLL_MS
#define HERMES_ADSB_TASK_POLL_MS         1000U
#endif
/* Tempo maximo esperando a fila de eventos (nao pode travar o resync). */
#ifndef HERMES_ADSB_QUEUE_TIMEOUT_MS
#define HERMES_ADSB_QUEUE_TIMEOUT_MS     100U
#endif

/* --------------------------------------------------------------- falha --- */
/* Sobrescreva se o projeto nao tiver Error_Handler(). */
#ifndef HERMES_ADSB_ON_ERROR
#define HERMES_ADSB_ON_ERROR()           Error_Handler()
#endif

#endif /* HERMES_CONFIG_H */

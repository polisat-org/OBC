/*
 * Enlace OBC <-> ADCS (Arduino Nano, repo polisat-org/ADCS, obc_link.h).
 *
 *   OBC -> ADCS: 1 byte com o VALOR do comando (não ASCII):
 *                1 inicia detumbling, 2 encerra, 3 inicia busca de azimute, 4 encerra
 *                5 reinicia o firmware do ADCS (desliga o motor antes)
 *   ADCS -> OBC: mensagens de 5 caracteres, sem terminador:
 *                azimute "ddd.d" ("270.5", "045.3"), 1 vez por segundo durante a busca, com luz;
 *                estado  "SDBF!" (D detumbling 0/1, B busca 0/1, F falha 0-4), logo depois de cada
 *                comando e a cada 2 s. Falha: 0 ok, 1 sobrevelocidade, 2 giroscópio, 3 ESCON,
 *                4 giroscópio ausente no boot.
 *
 * UART: 9600 8N1. PCB do OBC: USART2 (PA2 TX -> D12 do Nano, PA3 RX <- D11).
 * Bancada com o Nucleo: UART4 (A0 = PA0 TX, A1 = PA1 RX), porque a USART2 é o ST-Link.
 *
 * O parser procura o padrão ddd.d nos últimos 5 bytes recebidos (janela deslizante):
 * se o OBC começar a ouvir no meio de uma mensagem ou perder um byte, ele se realinha
 * sozinho na mensagem seguinte. Sem HAL: testado no PC.
 */
#ifndef ADCS_LINK_H
#define ADCS_LINK_H

#include <stdint.h>

#define ADCS_CMD_DETUMBLING_INICIA 1U
#define ADCS_CMD_DETUMBLING_PARA   2U
#define ADCS_CMD_AZIMUTE_INICIA    3U
#define ADCS_CMD_AZIMUTE_PARA      4U
#define ADCS_CMD_REINICIA          5U
#define ADCS_AZ_LEN                5U
#define ADCS_BAUD                  9600U

typedef enum { ADCS_MSG_NADA = 0, ADCS_MSG_AZIMUTE = 1, ADCS_MSG_ESTADO = 2 } AdcsMsgTipo;

typedef struct {
    uint8_t detumbling;   /* 0/1 */
    uint8_t busca;        /* 0/1 */
    uint8_t falha;        /* 0 ok; 1 sobrevelocidade; 2 giroscópio; 3 ESCON; 4 giroscópio ausente */
} AdcsEstado;

typedef struct {
    char janela[ADCS_AZ_LEN];
    uint8_t n;               /* bytes já na janela (até 5) */
    uint32_t aceitos;
    uint32_t descartados;    /* bytes que não fecharam um azimute válido */
} AdcsAzParser;

void adcs_az_init(AdcsAzParser *p);

/* Alimenta um byte. Devolve 1 e escreve o azimute (5 caracteres + '\0') em out quando
 * os últimos 5 bytes formam ddd.d entre 000.0 e 359.9. Pode ser chamada na ISR. */
int adcs_az_feed(AdcsAzParser *p, uint8_t byte, char out[ADCS_AZ_LEN + 1]);

/* Alimenta um byte e reconhece os dois tipos de mensagem. Devolve ADCS_MSG_AZIMUTE (az preenchido),
 * ADCS_MSG_ESTADO (est preenchido) ou ADCS_MSG_NADA. Pode ser chamada na ISR. */
AdcsMsgTipo adcs_feed(AdcsAzParser *p, uint8_t byte, char az[ADCS_AZ_LEN + 1], AdcsEstado *est);

/* Décimos de grau (0..3599) de um azimute já validado. */
uint16_t adcs_az_decimos(const char az[ADCS_AZ_LEN + 1]);

#endif /* ADCS_LINK_H */

/*
 * Downlink dos aviões do payload HERMES para o TT&C.
 *
 * O payload (HERMES Simples, Raspberry Pi) manda ao OBC 20 Plane_t de 204 bytes
 * a cada 5 s, sem cabeçalho e sem CRC (hermes_protocol.h). O enlace do TT&C é
 * LoRa a 9600, então o OBC repassa só o essencial de cada avião, num registro de
 * 36 bytes (os 28 B do Plane_t do ICD do payload + o indicativo de voo, 8 B):
 *
 *   quadro 'P' ao TT&C = 'P' + count (1 B) + seq (u16) + tx_met_ms (u32) + mensagens (u32)
 *                        + count x PlaneDown_t (36 B)                       (12 + 36n B)
 *
 *   seq        contador de quadros 'P' (perda no solo = buracos na sequência)
 *   tx_met_ms  instante da transmissão no relógio de missão do payload: maior met_s do bloco
 *              + tempo que o OBC segurou o bloco. Latência recepção -> transmissão de cada
 *              avião = tx_met_ms - last_pos_met_ms (HLR-ADS-08).
 *   mensagens  mensagens ADS-B decodificadas desde o início da missão (soma dos aumentos do
 *              contador "messages" do dump1090 de cada avião; ver pld_msgs_*).
 *
 * Como o bloco não tem CRC, cada registro passa por um teste de plausibilidade
 * antes de descer: ICAO de 24 bits inteiro, latitude/longitude na faixa.
 * Registros que falham são descartados e contados.
 *
 * Este módulo não usa HAL nem RTOS: é testado no PC (ver tests/ da bancada).
 */
#ifndef PAYLOAD_DOWNLINK_H
#define PAYLOAD_DOWNLINK_H

#include <stddef.h>
#include <stdint.h>

#include "hermes_protocol.h"

typedef struct __attribute__((packed)) {
    uint32_t icaoID;            /* 24 bits; nunca 0 no quadro 'P' */
    float    latitude;          /* graus; NaN se não houver posição */
    float    longitude;         /* graus; NaN se não houver posição */
    float    altitude_ft;       /* alt_baro, pés; NaN se ausente */
    float    velocity_kt;       /* gs, nós; NaN se ausente */
    float    track_deg;         /* track, graus; NaN se ausente */
    uint32_t last_pos_met_ms;   /* tempo de missão da posição (met_s - seen_pos), 0 se inválido */
    char     callsign[8];       /* flight do dump1090 (ASCII 0x20..0x7E), completado com 0; sem terminador se tiver 8 */
} PlaneDown_t;

_Static_assert(sizeof(PlaneDown_t) == 36U, "PlaneDown_t precisa ter 36 bytes");

#define PLD_DOWN_MAX_PLANES   HERMES_NUM_PLANES
#define PLD_DOWN_HDR_SIZE     12U
#define PLD_DOWN_FRAME_MAX    (PLD_DOWN_HDR_SIZE + PLD_DOWN_MAX_PLANES * sizeof(PlaneDown_t))   /* 732 B */
#define PLD_DOWN_MARKER       ((uint8_t)'P')

typedef struct {
    uint8_t validos;      /* aviões que vão no quadro 'P' */
    uint8_t vazios;       /* slots livres (icaoID == 0) */
    uint8_t descartados;  /* registros implausíveis (bloco corrompido?) */
    float   met_max_s;    /* maior met_s dos aviões válidos (instante do bloco); -1 se nenhum */
} PldCompactStats_t;

typedef struct {
    uint16_t seq;
    uint32_t tx_met_ms;
    uint32_t mensagens;
} PldDownHeader_t;

/* Contador de mensagens decodificadas na missão. O dump1090 conta as mensagens de cada avião
 * (campo messages, cumulativo); a cada bloco soma-se o aumento de cada ICAO em relação à última
 * vez que ele foi visto. Tabela de PLD_MSGS_TABELA ICAOs, substituição circular (o Pi emula igual). */
#define PLD_MSGS_TABELA 64U
typedef struct {
    uint32_t icao[PLD_MSGS_TABELA];
    uint32_t msgs[PLD_MSGS_TABELA];
    uint8_t  usados;
    uint8_t  proximo;
    uint32_t total;
} PldMsgs_t;

void     pld_msgs_reset(PldMsgs_t *m);
uint32_t pld_msgs_update(PldMsgs_t *m, const Plane_t in[HERMES_NUM_PLANES]);

/*
 * Converte o bloco do payload para registros de downlink.
 * Devolve quantos registros foram escritos em out (0..20).
 */
size_t pld_compact(const Plane_t in[HERMES_NUM_PLANES],
                   PlaneDown_t out[PLD_DOWN_MAX_PLANES],
                   PldCompactStats_t *stats);

/*
 * Monta o quadro 'P' (marcador + count + cabeçalho + registros) em buf, little-endian.
 * Devolve o tamanho do quadro, ou 0 se não couber em cap.
 */
size_t pld_build_frame(const PlaneDown_t *planes, size_t count, const PldDownHeader_t *hdr,
                       uint8_t *buf, size_t cap);

/* ------------------------------------------------------------ trailer TCS ---
 * 32 B no fim de cada bloco do Pi (hermes_simples/tcs_link.py), little-endian:
 *   0 "TCS1" | 4 versao | 5 flags (bit0 dado valido) | 6 idade (u16, 0,1 s; 0xFFFF sem dado)
 *   8 4 x int16 sondas 1..4 (0,01 C; -32768 invalida) | 16 duty A, duty B (%) | 18 situacao A, B
 *   20 t_s do TCS (u32, 0,1 s) | 24 missao ativa | 25 avioes | 26 reservado | 30 CRC16-CCITT de 0..29
 * Ao TT&C vai inteiro, num quadro 'H' = 'H' + 32 B. */
#define PLD_TCS_TEMP_INVALIDA  ((int16_t)-32768)
#define PLD_TCS_FRAME_SIZE     (1U + HERMES_TCS_SIZE)
#define PLD_TCS_MARKER         ((uint8_t)'H')

typedef struct {
    uint8_t  valido;        /* TCS rodando e linha com ate 10 s */
    uint16_t idade_ds;      /* 0xFFFF = sem dado */
    int16_t  t_x100[4];     /* sondas 1..4 */
    uint8_t  duty[2];       /* blocos A e B, % */
    uint8_t  situacao[2];   /* codigos do TCS */
    uint32_t t_ds;
    uint8_t  missao;
    uint8_t  avioes;
} PldTcs_t;

uint16_t pld_crc16(const uint8_t *data, size_t len);

/* Confere magic, versao e CRC. Devolve 1 e preenche out se o trailer e bom. */
int pld_tcs_decode(const uint8_t raw[HERMES_TCS_SIZE], PldTcs_t *out);

/* Menor temperatura valida (x100), ou PLD_TCS_TEMP_INVALIDA se nenhuma. */
int16_t pld_tcs_minima(const PldTcs_t *t);

/* Quadro 'H' ao TT&C: marcador + o trailer cru. Devolve o tamanho ou 0. */
size_t pld_build_tcs_frame(const uint8_t raw[HERMES_TCS_SIZE], uint8_t *buf, size_t cap);

#endif /* PAYLOAD_DOWNLINK_H */

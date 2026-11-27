/*
 * Downlink dos aviões do payload HERMES para o TT&C.
 *
 * O payload (HERMES Simples, Raspberry Pi) manda ao OBC 20 Plane_t de 204 bytes
 * a cada 5 s, sem cabeçalho e sem CRC (hermes_protocol.h). O enlace do TT&C é
 * LoRa a 9600, então o OBC repassa só o essencial de cada avião, num registro de
 * 36 bytes (os 28 B do Plane_t do ICD do payload + o indicativo de voo, 8 B):
 *
 *   quadro 'P' ao TT&C = 'P' + count (1 B) + count x PlaneDown_t (36 B)
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
#define PLD_DOWN_FRAME_MAX    (2U + PLD_DOWN_MAX_PLANES * sizeof(PlaneDown_t))   /* 722 B */
#define PLD_DOWN_MARKER       ((uint8_t)'P')

typedef struct {
    uint8_t validos;      /* aviões que vão no quadro 'P' */
    uint8_t vazios;       /* slots livres (icaoID == 0) */
    uint8_t descartados;  /* registros implausíveis (bloco corrompido?) */
} PldCompactStats_t;

/*
 * Converte o bloco do payload para registros de downlink.
 * Devolve quantos registros foram escritos em out (0..20).
 */
size_t pld_compact(const Plane_t in[HERMES_NUM_PLANES],
                   PlaneDown_t out[PLD_DOWN_MAX_PLANES],
                   PldCompactStats_t *stats);

/*
 * Monta o quadro 'P' (marcador + count + registros) em buf.
 * Devolve o tamanho do quadro, ou 0 se não couber em cap.
 */
size_t pld_build_frame(const PlaneDown_t *planes, size_t count,
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

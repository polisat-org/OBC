/*
 * Downlink dos aviões do payload HERMES para o TT&C.
 *
 * O payload (HERMES Simples, Raspberry Pi) manda ao OBC 20 Plane_t de 204 bytes
 * a cada 5 s, sem cabeçalho e sem CRC (hermes_protocol.h). O enlace do TT&C é
 * LoRa a 9600, então o OBC repassa só o essencial de cada avião, num registro de
 * 28 bytes (o Plane_t do ICD do payload, decidido para o downlink):
 *
 *   quadro 'P' ao TT&C = 'P' + count (1 B) + count x PlaneDown_t (28 B)
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
} PlaneDown_t;

_Static_assert(sizeof(PlaneDown_t) == 28U, "PlaneDown_t precisa ter 28 bytes");

#define PLD_DOWN_MAX_PLANES   HERMES_NUM_PLANES
#define PLD_DOWN_FRAME_MAX    (2U + PLD_DOWN_MAX_PLANES * sizeof(PlaneDown_t))   /* 562 B */
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

#endif /* PAYLOAD_DOWNLINK_H */

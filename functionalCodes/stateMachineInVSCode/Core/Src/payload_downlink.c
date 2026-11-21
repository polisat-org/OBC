/*
 * Downlink dos aviões do payload HERMES: Plane_t (204 B) -> PlaneDown_t (28 B).
 * Ver payload_downlink.h.
 */
#include "payload_downlink.h"

#include <math.h>
#include <string.h>

#define ICAO_MAX 16777215.0f   /* 24 bits */

static int finite_f(float v)
{
    return isfinite(v) ? 1 : 0;
}

/* ICAO vem como float (o "hex" do dump1090). Floats representam exatamente
 * inteiros de 24 bits, então um ICAO válido é inteiro, > 0 e <= 0xFFFFFF. */
static int icao_valido(float v, uint32_t *icao)
{
    if (!finite_f(v) || v < 1.0f || v > ICAO_MAX) {
        return 0;
    }
    uint32_t i = (uint32_t)v;
    if ((float)i != v) {
        return 0;
    }
    *icao = i;
    return 1;
}

static int posicao_plausivel(float lat, float lon)
{
    if (finite_f(lat) && (lat < -90.0f || lat > 90.0f)) {
        return 0;
    }
    if (finite_f(lon) && (lon < -180.0f || lon > 180.0f)) {
        return 0;
    }
    return 1;
}

size_t pld_compact(const Plane_t in[HERMES_NUM_PLANES],
                   PlaneDown_t out[PLD_DOWN_MAX_PLANES],
                   PldCompactStats_t *stats)
{
    PldCompactStats_t st = {0};
    size_t n = 0;

    for (size_t k = 0; k < HERMES_NUM_PLANES; k++) {
        const Plane_t *p = &in[k];
        uint32_t icao;

        if (p->icaoID == 0.0f) {
            st.vazios++;
            continue;
        }
        if (!icao_valido(p->icaoID, &icao) || !posicao_plausivel(p->lat, p->lon)) {
            st.descartados++;
            continue;
        }

        PlaneDown_t *d = &out[n++];
        const int tem_pos = finite_f(p->lat) && finite_f(p->lon);
        d->icaoID = icao;
        d->latitude = tem_pos ? p->lat : NAN;
        d->longitude = tem_pos ? p->lon : NAN;
        d->altitude_ft = finite_f(p->alt_baro) ? p->alt_baro : NAN;
        d->velocity_kt = finite_f(p->gs) ? p->gs : NAN;
        d->track_deg = finite_f(p->track) ? p->track : NAN;

        /* Tempo da posição = momento em que o payload processou - idade da posição. */
        d->last_pos_met_ms = 0U;
        if (tem_pos && finite_f(p->met_s) && finite_f(p->seen_pos)
                && p->met_s >= 0.0f && p->seen_pos >= 0.0f && p->seen_pos <= p->met_s) {
            d->last_pos_met_ms = (uint32_t)((p->met_s - p->seen_pos) * 1000.0f);
        }
    }

    st.validos = (uint8_t)n;
    if (stats != NULL) {
        *stats = st;
    }
    return n;
}

size_t pld_build_frame(const PlaneDown_t *planes, size_t count,
                       uint8_t *buf, size_t cap)
{
    if (count > PLD_DOWN_MAX_PLANES) {
        return 0U;
    }
    size_t len = 2U + count * sizeof(PlaneDown_t);
    if (buf == NULL || cap < len || (count > 0U && planes == NULL)) {
        return 0U;
    }
    buf[0] = PLD_DOWN_MARKER;
    buf[1] = (uint8_t)count;
    if (count > 0U) {
        memcpy(&buf[2], planes, count * sizeof(PlaneDown_t));
    }
    return len;
}

/* ------------------------------------------------------------ trailer TCS --- */
uint16_t pld_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFU;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static uint16_t le16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t le32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }

int pld_tcs_decode(const uint8_t raw[HERMES_TCS_SIZE], PldTcs_t *out)
{
    if (raw == NULL || out == NULL) {
        return 0;
    }
    if (raw[0] != 'T' || raw[1] != 'C' || raw[2] != 'S' || raw[3] != '1' || raw[4] != 1U) {
        return 0;
    }
    if (pld_crc16(raw, 30U) != le16(&raw[30])) {
        return 0;
    }
    out->valido = (uint8_t)(raw[5] & 1U);
    out->idade_ds = le16(&raw[6]);
    for (int k = 0; k < 4; k++) {
        out->t_x100[k] = (int16_t)le16(&raw[8 + 2 * k]);
    }
    out->duty[0] = raw[16];
    out->duty[1] = raw[17];
    out->situacao[0] = raw[18];
    out->situacao[1] = raw[19];
    out->t_ds = le32(&raw[20]);
    out->missao = raw[24];
    out->avioes = raw[25];
    return 1;
}

int16_t pld_tcs_minima(const PldTcs_t *t)
{
    int16_t m = PLD_TCS_TEMP_INVALIDA;
    for (int k = 0; k < 4; k++) {
        if (t->t_x100[k] != PLD_TCS_TEMP_INVALIDA && (m == PLD_TCS_TEMP_INVALIDA || t->t_x100[k] < m)) {
            m = t->t_x100[k];
        }
    }
    return m;
}

size_t pld_build_tcs_frame(const uint8_t raw[HERMES_TCS_SIZE], uint8_t *buf, size_t cap)
{
    if (raw == NULL || buf == NULL || cap < PLD_TCS_FRAME_SIZE) {
        return 0U;
    }
    buf[0] = PLD_TCS_MARKER;
    memcpy(&buf[1], raw, HERMES_TCS_SIZE);
    return PLD_TCS_FRAME_SIZE;
}

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "hermes_protocol.h"
#include "payload_downlink.h"
#define CHECK(c) do { if (!(c)) { printf("FALHOU linha %d: %s\n", __LINE__, #c); return 1; } } while (0)
int main(int argc, char **argv) { (void)argc;
    FILE *f = fopen(argv[1], "rb"); uint8_t raw[HERMES_BUFFER_SIZE];
    CHECK(f && fread(raw, 1, sizeof raw, f) == sizeof raw); fclose(f);
    /* passa os bytes pelo receptor real do OBC, byte a byte como a ISR */
    static hermes_rx_t rx; hermes_rx_init(&rx);
    hermes_rx_feed_bytes(&rx, raw, sizeof raw);
    CHECK(hermes_rx_complete(&rx));
    Plane_t in[HERMES_NUM_PLANES]; hermes_rx_take(&rx, in);
    PlaneDown_t out[PLD_DOWN_MAX_PLANES]; PldCompactStats_t st;
    size_t n = pld_compact(in, out, &st);
    printf("validos=%u vazios=%u descartados=%u\n", st.validos, st.vazios, st.descartados);
    CHECK(n == 3 && st.validos == 3 && st.descartados == 2 && st.vazios == 15);
    CHECK(out[0].icaoID == 0xE48C01);
    CHECK(fabsf(out[0].latitude + 23.55f) < 1e-4f && fabsf(out[0].longitude + 46.63f) < 1e-4f);
    CHECK(out[0].altitude_ft == 35000.0f && fabsf(out[0].velocity_kt - 450.5f) < 1e-3f && out[0].track_deg == 90.0f);
    CHECK(out[0].last_pos_met_ms == 11000);            /* 12.5 s - 1.5 s */
    CHECK(memcmp(out[0].callsign, "TAM3054 ", 8) == 0);
    CHECK(out[1].icaoID == 0x4B1817 && out[1].last_pos_met_ms == 20000);
    CHECK(memcmp(out[1].callsign, "GLO12\0\0\0", 8) == 0);   /* para no byte 0x01 */
    CHECK(out[2].icaoID == 0xABCDEF && isnan(out[2].latitude) && isnan(out[2].longitude));
    CHECK(out[2].last_pos_met_ms == 0 && out[2].altitude_ft == 15000.0f && isnan(out[2].track_deg));
    CHECK(memcmp(out[2].callsign, "\0\0\0\0\0\0\0\0", 8) == 0);   /* sem indicativo */
    uint8_t q[PLD_DOWN_FRAME_MAX];
    size_t len = pld_build_frame(out, n, q, sizeof q);
    CHECK(len == 2 + 3 * 36 && q[0] == 'P' && q[1] == 3);
    CHECK(pld_build_frame(out, n, q, 10) == 0);        /* não cabe */
    CHECK(pld_build_frame(NULL, 0, q, sizeof q) == 2); /* quadro vazio */
    FILE *o = fopen(argv[2], "wb"); fwrite(q, 1, len, o); fclose(o);
    /* trailer TCS gerado pelo hermes_simples/tcs_link.py real (vetor de referência) */
    static const uint8_t tcsRef[HERMES_TCS_SIZE] = { 0x54, 0x43, 0x53, 0x31, 0x01, 0x01, 0x0F, 0x00, 0x02, 0x08, 0xBB, 0xFE, 0xB7, 0x07, 0x00, 0x80, 0x23, 0x64, 0x00, 0x06, 0xFD, 0x3A, 0x00, 0x00, 0x01, 0x07, 0x00, 0x00, 0x00, 0x00, 0xB3, 0xE4 };
    PldTcs_t t;
    CHECK(pld_crc16((const uint8_t *)"123456789", 9) == 0x29B1);
    CHECK(pld_tcs_decode(tcsRef, &t) == 1);
    CHECK(t.valido == 1 && t.idade_ds == 15);
    CHECK(t.t_x100[0] == 2050 && t.t_x100[1] == -325 && t.t_x100[2] == 1975 && t.t_x100[3] == PLD_TCS_TEMP_INVALIDA);
    CHECK(t.duty[0] == 35 && t.duty[1] == 100 && t.situacao[0] == 0 && t.situacao[1] == 6);
    CHECK(t.t_ds == 15101 && t.missao == 1 && t.avioes == 7);
    CHECK(pld_tcs_minima(&t) == -325);
    uint8_t ruim[HERMES_TCS_SIZE]; memcpy(ruim, tcsRef, sizeof ruim); ruim[9] ^= 0x40;
    CHECK(pld_tcs_decode(ruim, &t) == 0);                 /* CRC pega a corrupção */
    uint8_t h[PLD_TCS_FRAME_SIZE];
    CHECK(pld_build_tcs_frame(tcsRef, h, sizeof h) == 33 && h[0] == 'H' && memcmp(&h[1], tcsRef, 32) == 0);
    printf("quadro 'P' = %zu B; trailer TCS ok; todos os testes OK\n", len);
    return 0;
}

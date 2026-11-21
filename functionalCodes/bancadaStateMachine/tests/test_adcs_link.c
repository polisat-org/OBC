/* Teste no PC do parser do azimute do ADCS (adcs_link.c). */
#include <stdio.h>
#include <string.h>
#include "adcs_link.h"
#define CHECK(c) do { if (!(c)) { printf("FALHOU linha %d: %s\n", __LINE__, #c); return 1; } } while (0)

static int alimenta(AdcsAzParser *p, const char *s, char ult[6]) {
    int n = 0; char out[6];
    for (size_t i = 0; s[i]; i++) if (adcs_az_feed(p, (uint8_t)s[i], out)) { n++; memcpy(ult, out, 6); }
    return n;
}

int main(void) {
    AdcsAzParser p; char az[6];
    adcs_az_init(&p);
    CHECK(alimenta(&p, "270.5", az) == 1 && strcmp(az, "270.5") == 0);
    CHECK(adcs_az_decimos(az) == 2705);
    CHECK(alimenta(&p, "045.3000.0359.9", az) == 3 && strcmp(az, "359.9") == 0);
    /* começou a ouvir no meio: "0.5" + mensagens seguintes realinham */
    adcs_az_init(&p);
    CHECK(alimenta(&p, "0.5270.4", az) == 1 && strcmp(az, "270.4") == 0);
    /* byte perdido: "27.5" (sem o 0) e depois mensagem boa */
    adcs_az_init(&p);
    CHECK(alimenta(&p, "27.5045.3", az) == 1 && strcmp(az, "045.3") == 0);
    /* fora da faixa e lixo não passam */
    adcs_az_init(&p);
    CHECK(alimenta(&p, "360.0", az) == 0);
    CHECK(alimenta(&p, "\x01\x02\x03\x04" "12.45", az) == 0);
    CHECK(alimenta(&p, "123.4", az) == 1 && strcmp(az, "123.4") == 0);
    /* mensagem de estado SDBF! intercalada com azimutes */
    {
        AdcsAzParser q; AdcsEstado e = {9, 9, 9}; char a[6]; const char *fluxo = "S100!270.5S014!x0S01";
        int nEst = 0, nAz = 0;
        adcs_az_init(&q);
        for (size_t i = 0; fluxo[i]; i++) {
            AdcsMsgTipo t2 = adcs_feed(&q, (uint8_t)fluxo[i], a, &e);
            if (t2 == ADCS_MSG_ESTADO) nEst++;
            if (t2 == ADCS_MSG_AZIMUTE) nAz++;
        }
        CHECK(nEst == 2 && nAz == 1);
        CHECK(e.detumbling == 0 && e.busca == 1 && e.falha == 4);
        adcs_az_init(&q);
        CHECK(adcs_feed(&q, 'S', a, &e) == ADCS_MSG_NADA);
        for (const char *z = "1251"; *z; z++) adcs_feed(&q, (uint8_t)*z, a, &e);   /* falha 5 inválida */
        CHECK(e.falha == 4);
    }
    printf("parser do azimute: todos os testes OK (%lu aceitos, %lu bytes descartados)\n",
           (unsigned long)p.aceitos, (unsigned long)p.descartados);
    return 0;
}

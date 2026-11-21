/* Enlace OBC <-> ADCS: parser do azimute. Ver adcs_link.h. */
#include "adcs_link.h"

#include <stddef.h>

static int digito(char c) { return c >= '0' && c <= '9'; }

void adcs_az_init(AdcsAzParser *p)
{
    p->n = 0U;
    p->aceitos = 0U;
    p->descartados = 0U;
}

static int bit01(char c) { return c == '0' || c == '1'; }

AdcsMsgTipo adcs_feed(AdcsAzParser *p, uint8_t byte, char az[ADCS_AZ_LEN + 1], AdcsEstado *est)
{
    if (p->n == ADCS_AZ_LEN) {
        /* janela cheia: desliza um byte (o mais antigo não fechou azimute) */
        for (uint8_t k = 0U; k + 1U < ADCS_AZ_LEN; k++) {
            p->janela[k] = p->janela[k + 1U];
        }
        p->n--;
        p->descartados++;
    }
    p->janela[p->n++] = (char)byte;
    if (p->n < ADCS_AZ_LEN) {
        return ADCS_MSG_NADA;
    }
    const char *j = p->janela;
    if (j[0] == 'S' && bit01(j[1]) && bit01(j[2]) && j[3] >= '0' && j[3] <= '4' && j[4] == '!') {
        if (est != NULL) {
            est->detumbling = (uint8_t)(j[1] - '0');
            est->busca = (uint8_t)(j[2] - '0');
            est->falha = (uint8_t)(j[3] - '0');
        }
        p->n = 0U;
        p->aceitos++;
        return ADCS_MSG_ESTADO;
    }
    if (!(digito(j[0]) && digito(j[1]) && digito(j[2]) && j[3] == '.' && digito(j[4]))) {
        return ADCS_MSG_NADA;
    }
    unsigned graus = (unsigned)(j[0] - '0') * 100U + (unsigned)(j[1] - '0') * 10U + (unsigned)(j[2] - '0');
    if (graus > 359U) {
        return ADCS_MSG_NADA;
    }
    if (az != NULL) {
        for (uint8_t k = 0U; k < ADCS_AZ_LEN; k++) {
            az[k] = j[k];
        }
        az[ADCS_AZ_LEN] = '\0';
    }
    p->n = 0U;            /* consumiu a mensagem inteira */
    p->aceitos++;
    return ADCS_MSG_AZIMUTE;
}

int adcs_az_feed(AdcsAzParser *p, uint8_t byte, char out[ADCS_AZ_LEN + 1])
{
    return adcs_feed(p, byte, out, NULL) == ADCS_MSG_AZIMUTE;
}

uint16_t adcs_az_decimos(const char az[ADCS_AZ_LEN + 1])
{
    return (uint16_t)((az[0] - '0') * 1000 + (az[1] - '0') * 100 + (az[2] - '0') * 10 + (az[4] - '0'));
}

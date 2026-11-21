#include "hermes_protocol.h"

#include <string.h>

void hermes_rx_init(hermes_rx_t *rx) {
    /* Funcao de init: nunca chamar dentro da ISR (faz memset de 4080 bytes). */
    rx->index = 0;
    rx->complete = 0;
    rx->bytes_received = 0;
    rx->overruns = 0;
    memset(rx->bytes, 0, sizeof(rx->bytes));
}

/* Isto e o que o HAL_UART_RxCpltCallback chama, byte a byte. */
void hermes_rx_on_byte_isr(hermes_rx_t *rx, uint8_t byte) {
    if (rx->complete) {
        /* Chegou byte de um proximo buffer antes do main chamar take(). */
        rx->overruns++;
        return;
    }
    if (rx->index >= HERMES_BUFFER_SIZE) {
        /* Defensiva: nao deve acontecer, mas protege o array. */
        rx->overruns++;
        rx->complete = 1;
        return;
    }

    rx->bytes[rx->index++] = byte;
    rx->bytes_received++;
    if (rx->index >= HERMES_BUFFER_SIZE) {
        rx->complete = 1;
    }
}

void hermes_rx_feed_byte(hermes_rx_t *rx, uint8_t byte) {
    hermes_rx_on_byte_isr(rx, byte);
}

void hermes_rx_feed_bytes(hermes_rx_t *rx, const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        hermes_rx_on_byte_isr(rx, data[i]);
    }
}

int hermes_rx_complete(const hermes_rx_t *rx) {
    return rx->complete != 0;
}

uint32_t hermes_rx_overruns(const hermes_rx_t *rx) {
    return rx->overruns;
}

/* Fora da ISR. No HAL real, mascarar a IRQ da UART em volta desta chamada. */
void hermes_rx_take(hermes_rx_t *rx, Plane_t out[HERMES_NUM_PLANES]) {
    if (out != NULL) {
        memcpy(out, rx->bytes, HERMES_BUFFER_SIZE);
    }
    hermes_rx_init(rx);
}

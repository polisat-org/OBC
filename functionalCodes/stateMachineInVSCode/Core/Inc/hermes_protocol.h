/*
 * HERMES Simples - protocolo UART bruto (sem cabecalho, sem CRC).
 * OBC recebe 20 registros de 204 bytes = 4080 bytes a cada 5 s.
 * Tudo little-endian; floats IEEE-754 de 32 bits.
 *
 * MODELO DE RECEPCAO: UART RX INTERRUPT.
 * O HAL chama hermes_rx_on_byte_isr() uma vez por byte recebido; o loop
 * principal so chama hermes_rx_take() quando hermes_rx_complete() e vero.
 */
#ifndef HERMES_PROTOCOL_H
#define HERMES_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#define HERMES_NUM_PLANES 20u
#define HERMES_PLANE_SIZE 204u
#define HERMES_BUFFER_SIZE 4080u
#define HERMES_SEND_PERIOD_S 5u

#define HFLAG_MODE_A   (1u << 0)
#define HFLAG_MODE_C   (1u << 1)
#define HFLAG_MLAT     (1u << 2)
#define HFLAG_TISB     (1u << 3)

#define HNAV_AUTOPILOT (1u << 0)
#define HNAV_VNAV      (1u << 1)
#define HNAV_ALT_HOLD  (1u << 2)
#define HNAV_APPROACH  (1u << 3)
#define HNAV_LNAV      (1u << 4)
#define HNAV_TCAS      (1u << 5)

typedef struct {
    char     flight[8];
    char     squawk[8];

    float icaoID;
    float met_s;        /* timestamp: segundos desde o inicio da missao */
    float addrtype;
    float alt_baro;
    float alt_geom;
    float gs;
    float ias;
    float tas;
    float mach;
    float track;
    float track_rate;
    float roll;
    float mag_heading;
    float true_heading;
    float baro_rate;
    float geom_rate;
    float category;
    float nav_qnh;
    float nav_altitude_mcp;
    float nav_altitude_fms;
    float nav_heading;
    float lat;
    float lon;
    float nic;
    float rc;
    float seen_pos;
    float version;
    float nic_baro;
    float nac_p;
    float nac_v;
    float sil;
    float sil_type;
    float gva;
    float sda;
    float mrar_source;
    float wind_speed;
    float wind_dir;
    float temperature;
    float pressure;
    float turbulence;
    float humidity;
    float emergency;
    float messages;
    float seen;
    float rssi;

    uint32_t nav_modes;
    uint32_t flags;
} Plane_t;

_Static_assert(sizeof(Plane_t) == HERMES_PLANE_SIZE,
               "Plane_t precisa ter exatamente 204 bytes");
_Static_assert(HERMES_PLANE_SIZE * HERMES_NUM_PLANES == HERMES_BUFFER_SIZE,
               "buffer precisa ser 20 * 204 = 4080 bytes");

/*
 * Estado compartilhado entre a ISR da UART e o loop principal.
 * index/complete/contadores sao volatile porque a ISR escreve e o main le.
 * buf nao precisa de volatile: so e copiado depois de complete == 1.
 */
typedef struct {
    uint8_t  bytes[HERMES_BUFFER_SIZE];
    volatile uint32_t index;
    volatile uint32_t complete;
    volatile uint32_t bytes_received;
    volatile uint32_t overruns;
} hermes_rx_t;

/* Chamar no init, fora da ISR. */
void hermes_rx_init(hermes_rx_t *rx);

/* CHAMAR DENTRO DO CALLBACK DE RX INTERRUPT, UMA VEZ POR BYTE. */
void hermes_rx_on_byte_isr(hermes_rx_t *rx, uint8_t byte);

/* Alias usado pelos testes host; chama hermes_rx_on_byte_isr(). */
void hermes_rx_feed_byte(hermes_rx_t *rx, uint8_t byte);

/* Helper so para testes/bancada (nao usar no STM32). */
void hermes_rx_feed_bytes(hermes_rx_t *rx, const uint8_t *data, size_t len);

int  hermes_rx_complete(const hermes_rx_t *rx);
uint32_t hermes_rx_overruns(const hermes_rx_t *rx);

/*
 * Copia os 20 avioes e reinicia o receptor para o proximo envio.
 * No STM32, desabilite a IRQ da UART antes de chamar e reabilite depois.
 */
void hermes_rx_take(hermes_rx_t *rx, Plane_t out[HERMES_NUM_PLANES]);

#endif

#ifndef CUSTOM_TYPES_H
#define CUSTOM_TYPES_H

#include <stdint.h>

typedef struct Telemetry Telemetry_t;

typedef enum {
	EVT_HK_DONE,
	EVT_RX_DONE,
	EVT_TX_DONE,
	EVT_MISSION_DONE,
	EVT_DEPLOY_DONE,
	EVT_DETUMBLING_DONE,
} EventType_t;

typedef union {
	uint8_t telecommand;
	Telemetry_t *telemetry;
	uint32_t error_code;
} EventData_t;

typedef struct {
	EventType_t type;
	uint32_t timestamp;
	EventData_t data;
} Event_t;

typedef enum {
	SAT_MODE_IDLE,
	SAT_MODE_MISSION,
	SAT_MODE_DETUMBLING,
} SatMode_t;

typedef enum {
	TC_START_MISSION = 0x01,
	TC_DEPLOY = 0x02,
	TC_START_DETUMBLING = 0x04,
} Telecommand_t;

struct __attribute__((packed)) Telemetry {
    // OBC
    uint8_t hora;
    uint8_t minuto;
    uint8_t segundo;
    uint8_t dia;
    uint8_t mes;
    uint8_t ano;

    // Mecanica
    float temperatura_bateria;
    float temperatura_estrutura;
    float umidade;
    float pressao;
    uint8_t deploy_antena;

    // EPS
    // Tensao e corrente 13.8V e 5V
    float tensao_13v8;
    float tensao_5v;
    float corrente_13v8;
    float corrente_5v;

    // Paineis Solares
    float tensao_painel_solar_1;
    float tensao_painel_solar_2;
    float tensao_painel_solar_3;
    float tensao_painel_solar_4;

    // Status e Falhas dos Carregadores
    uint8_t status_carregador_1;
    uint8_t status_carregador_2;
    uint8_t falha_carregador_1;
    uint8_t falha_carregador_2;

    // ADC
    uint8_t adc_tensao_sistema_1;
    uint8_t adc_tensao_sistema_2;
    uint8_t adc_tensao_ntc_1;
    uint8_t adc_tensao_ntc_2;
    uint8_t adc_corrente_carga_1;
    uint8_t adc_corrente_carga_2;
    uint8_t adc_corrente_sistema_1;
    uint8_t adc_corrente_sistema_2;

    char angulo_apontamento[6];

    // Baterias
    uint16_t tensao_bateria_1;
    uint16_t tensao_bateria_2;
    uint16_t carga_bateria_1;
    uint16_t carga_bateria_2;
    int16_t  taxa_carga_descarga_bateria_1;
    int16_t  taxa_carga_descarga_bateria_2;

    // Carga Util

};

_Static_assert(sizeof(Telemetry_t) == 85U,
		"Telemetry_t changed: update TELEMETRY_FORMAT in uart.py");

#endif

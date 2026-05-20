#ifndef CUSTOM_TYPES_H
#define CUSTOM_TYPES_H

typedef enum {
	EVT_INIT_DONE,
	EVT_TC_SLOT,
	EVT_HK_TICK,
	EVT_HK_DONE,
	EVT_TX_DONE
} EventType_t;

typedef struct {
	EventType_t type;
	uint32_t timestamp;
	void *payload;
} Event_t;

typedef enum {
	STATE_INIT,
	STATE_IDLE,
	STATE_RECEIVE,
	STATE_HOUSEKEEP,
	STATE_TRANSMIT
} SatState_t;

typedef struct __attribute__((packed)) {
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

    // Baterias
    uint16_t tensao_bateria_1;
    uint16_t tensao_bateria_2;
    uint16_t carga_bateria_1;
    uint16_t carga_bateria_2;
    int16_t  taxa_carga_descarga_bateria_1;
    int16_t  taxa_carga_descarga_bateria_2;
} Telemetry_t;

#endif

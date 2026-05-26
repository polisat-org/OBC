#include <stdio.h>
#include "dataReadying.h"

void putUint8(uint8_t *buffer, uint8_t *index, uint8_t value) {
    // Coloca value em buffer[index] e aumenta index
    buffer[*index] = value;
    (*index)++;
    return;
}
void putFloat(uint8_t *buffer, uint8_t *index, float value) {
    uint16_t fullValue = (int16_t) (value * 100); // float AB.CD vira ABCD
    // Se o dado do float for negativo, conversao de volta para int16_t mostra valor original
    putUint16(buffer, index, fullValue); // Coloca no buffer como dois bytes 
    return;
}

void putUint16(uint8_t *buffer, uint8_t *index, uint16_t value) { 
    // value ABCD
    buffer[*index] = value & 0xFF; // CD em buffer[index] primeiro, stm32 eh Little Endian
    (*index)++;
    buffer[*index] = (value >> 8) & 0xFF; // AB em buffer[index+1]
    (*index)++;
    return;
}

uint8_t getUint8(uint8_t *buffer, uint8_t *index) {
    // Retorna o que esta em buffer[index] e aumenta index
    uint8_t value = buffer[*index];
    (*index)++;
    return value;
}

uint16_t getUint16(uint8_t *buffer, uint8_t *index) {
    // Pega CD em buffer[index] e AB em buffer[index+1], junta como ABCD e aumenta index em 2
    uint16_t value = buffer[*index];
    (*index)++;
    value |= ((uint16_t)buffer[*index] << 8);
    (*index)++;
    return value;
}

float getFloat(uint8_t *buffer, uint8_t *index) {
    // Pega o valor inteiro de 16 bits e converte para float
    int16_t fullValue = (int16_t)getUint16(buffer, index); // Pega sinal, pode ser negativo
    return fullValue / 100.0f; // ABCD -> AB.CD
}

int readyData(telemetryFull *dados, uint8_t *buffer) {
    uint8_t index = 0;
    
    // OBC
    putUint8(buffer, &index, dados->hora);
    putUint8(buffer, &index, dados->minuto);
    putUint8(buffer, &index, dados->segundo);
    putUint8(buffer, &index, dados->dia);
    putUint8(buffer, &index, dados->mes);
    putUint8(buffer, &index, dados->ano);

    // Mecanica
    putFloat(buffer, &index, dados->temperatura_bateria);
    putFloat(buffer, &index, dados->temperatura_externa);
    putUint8(buffer, &index, dados->deploy_antena);

    // EPS
    // Tensao e corrente 13.8V e 5V
    putFloat(buffer, &index, dados->tensao_13v8);
    putFloat(buffer, &index, dados->tensao_5v);
    putFloat(buffer, &index, dados->corrente_13v8);
    putFloat(buffer, &index, dados->corrente_5v);

    // Paineis Solares
    putFloat(buffer, &index, dados->tensao_painel_solar_1);
    putFloat(buffer, &index, dados->tensao_painel_solar_2);
    putFloat(buffer, &index, dados->tensao_painel_solar_3);
    putFloat(buffer, &index, dados->tensao_painel_solar_4);

    // Status e Falhas dos Carregadores
    putUint8(buffer, &index, dados->status_carregador_1);
    putUint8(buffer, &index, dados->status_carregador_2);
    putUint8(buffer, &index, dados->falha_carregador_1);
    putUint8(buffer, &index, dados->falha_carregador_2);

    // ADC
    putUint8(buffer, &index, dados->adc_tensao_sistema_1);
    putUint8(buffer, &index, dados->adc_tensao_sistema_2);
    putUint8(buffer, &index, dados->adc_tensao_ntc_1);
    putUint8(buffer, &index, dados->adc_tensao_ntc_2);
    putUint8(buffer, &index, dados->adc_corrente_carga_1);
    putUint8(buffer, &index, dados->adc_corrente_carga_2);
    putUint8(buffer, &index, dados->adc_corrente_sistema_1);
    putUint8(buffer, &index, dados->adc_corrente_sistema_2);

    // Baterias 
    putUint16(buffer, &index, dados->tensao_bateria_1);
    putUint16(buffer, &index, dados->tensao_bateria_2);
    putUint16(buffer, &index, dados->carga_bateria_1);
    putUint16(buffer, &index, dados->carga_bateria_2);
    putUint16(buffer, &index, (uint16_t) dados->taxa_carga_descarga_bateria_1);
    putUint16(buffer, &index, (uint16_t) dados->taxa_carga_descarga_bateria_2);

    return index;
}

void readyDataMay26(telemetryMay26 *dados, uint8_t *buffer)
{
  buffer[0] = dados->hora;
  buffer[1] = dados->minuto;
  buffer[2] = dados->segundo;
  buffer[3] = dados->dia;
  buffer[4] = dados->mes;
  buffer[5] = dados->ano;
  buffer[6] = dados->temperatura_bateria;
  buffer[7] = (dados->temperatura_bateria - buffer[6]) * 100;

  return;
}

int parseData(telemetryFull *dados, uint8_t *buffer) {
    uint8_t index = 0;

    // OBC
    dados->hora = getUint8(buffer, &index);
    dados->minuto = getUint8(buffer, &index);
    dados->segundo = getUint8(buffer, &index);
    dados->dia = getUint8(buffer, &index);
    dados->mes = getUint8(buffer, &index);
    dados->ano = getUint8(buffer, &index);

    // Mecanica
    dados->temperatura_bateria = getFloat(buffer, &index);
    dados->temperatura_externa = getFloat(buffer, &index);
    dados->deploy_antena = getUint8(buffer, &index);

    // EPS
    dados->tensao_13v8 = getFloat(buffer, &index);
    dados->tensao_5v = getFloat(buffer, &index);
    dados->corrente_13v8 = getFloat(buffer, &index);
    dados->corrente_5v = getFloat(buffer, &index);

    // Paineis Solares
    dados->tensao_painel_solar_1 = getFloat(buffer, &index);
    dados->tensao_painel_solar_2 = getFloat(buffer, &index);
    dados->tensao_painel_solar_3 = getFloat(buffer, &index);
    dados->tensao_painel_solar_4 = getFloat(buffer, &index);

    // Status e Falhas dos Carregadores
    dados->status_carregador_1 = getUint8(buffer, &index);
    dados->status_carregador_2 = getUint8(buffer, &index);
    dados->falha_carregador_1 = getUint8(buffer, &index);
    dados->falha_carregador_2 = getUint8(buffer, &index);

    // ADC
    dados->adc_tensao_sistema_1 = getUint8(buffer, &index);
    dados->adc_tensao_sistema_2 = getUint8(buffer, &index);
    dados->adc_tensao_ntc_1 = getUint8(buffer, &index);
    dados->adc_tensao_ntc_2 = getUint8(buffer, &index);
    dados->adc_corrente_carga_1 = getUint8(buffer, &index);
    dados->adc_corrente_carga_2 = getUint8(buffer, &index);
    dados->adc_corrente_sistema_1 = getUint8(buffer, &index);
    dados->adc_corrente_sistema_2 = getUint8(buffer, &index);

    // Baterias 
    dados->tensao_bateria_1 = getUint16(buffer, &index);
    dados->tensao_bateria_2 = getUint16(buffer, &index);
    dados->carga_bateria_1 = getUint16(buffer, &index);
    dados->carga_bateria_2 = getUint16(buffer, &index);
    dados->taxa_carga_descarga_bateria_1 = (int16_t)getUint16(buffer, &index);
    dados->taxa_carga_descarga_bateria_2 = (int16_t)getUint16(buffer, &index);

    return index;
}

void printTelemetry(telemetryFull *dados)
{
    printf("Hora: %d\n", dados->hora);
    printf("Minuto: %d\n", dados->minuto);
    printf("Segundo: %d\n", dados->segundo);
    printf("Dia: %d\n", dados->dia);
    printf("Mês: %d\n", dados->mes);
    printf("Ano: %d\n", dados->ano);

    printf("Temperatura da Bateria: %.2f\n", dados->temperatura_bateria);
    printf("Temperatura Externa: %.2f\n", dados->temperatura_externa);
    printf("Deploy Antena: %d\n", dados->deploy_antena);

    printf("Tensão 13.8V: %.2f\n", dados->tensao_13v8);
    printf("Tensão 5V: %.2f\n", dados->tensao_5v);
    printf("Corrente 13.8V: %.2f\n", dados->corrente_13v8);
    printf("Corrente 5V: %.2f\n", dados->corrente_5v);

    printf("Tensão Painel Solar 1: %.2f\n", dados->tensao_painel_solar_1);
    printf("Tensão Painel Solar 2: %.2f\n", dados->tensao_painel_solar_2);
    printf("Tensão Painel Solar 3: %.2f\n", dados->tensao_painel_solar_3);
    printf("Tensão Painel Solar 4: %.2f\n", dados->tensao_painel_solar_4);

    printf("Status Carregador 1: %d\n", dados->status_carregador_1);
    printf("Status Carregador 2: %d\n", dados->status_carregador_2);
    printf("Falha Carregador 1: %d\n", dados->falha_carregador_1);
    printf("Falha Carregador 2: %d\n", dados->falha_carregador_2);

    printf("ADC Tensão Sistema 1: %d\n", dados->adc_tensao_sistema_1);
    printf("ADC Tensão Sistema 2: %d\n", dados->adc_tensao_sistema_2);
    printf("ADC Tensão NTC 1: %d\n", dados->adc_tensao_ntc_1);
    printf("ADC Tensão NTC 2: %d\n", dados->adc_tensao_ntc_2);
    printf("ADC Corrente Carga 1: %d\n", dados->adc_corrente_carga_1);
    printf("ADC Corrente Carga 2: %d\n", dados->adc_corrente_carga_2);
    printf("ADC Corrente Sistema 1: %d\n", dados->adc_corrente_sistema_1);
    printf("ADC Corrente Sistema 2: %d\n", dados->adc_corrente_sistema_2);

    printf("Tensão Bateria 1: %u\n", dados->tensao_bateria_1);
    printf("Tensão Bateria 2: %u\n", dados->tensao_bateria_2);
    printf("Carga Bateria 1: %u\n", dados->carga_bateria_1);
    printf("Carga Bateria 2: %u\n", dados->carga_bateria_2);
    printf("Taxa Carga/Descarga Bateria 1: %d\n", dados->taxa_carga_descarga_bateria_1);
    printf("Taxa Carga/Descarga Bateria 2: %d\n", dados->taxa_carga_descarga_bateria_2);
}

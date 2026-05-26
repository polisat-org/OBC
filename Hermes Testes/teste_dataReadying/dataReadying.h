#include "telemetryType.h"

// Funcoes para colocar valores no buffer
void putUint8(uint8_t *buffer, uint8_t *index, uint8_t value);
void putFloat(uint8_t *buffer, uint8_t *index, float value);
void putUint16(uint8_t *buffer, uint8_t *index, uint16_t value);

// Funcoes para ler valores do buffer
uint8_t getUint8(uint8_t *buffer, uint8_t *index);
uint16_t getUint16(uint8_t *buffer, uint8_t *index);
float getFloat(uint8_t *buffer, uint8_t *index);

// Funcoes para manipular struct inteira no buffer
int readyData(telemetryFull *dados, uint8_t *buffer); // Coloca dados em buffer, retorna tamanho buffer
void readyDataMay26(telemetryMay26 *dados, uint8_t *buffer);
int parseData(telemetryFull *dados, uint8_t *buffer); // Coloca buffer em dados, retorna tamanho buffer

// Printf
void printTelemetry(telemetryFull *dados);



/* EPS na telemetria: carregadores MP2733, medidores MAX17048 e monitores INA3221 atrás do MUX I2C PCA9548A.
 * Drivers, endereços, canais do MUX e mapeamento para a telemetria são os do firmware da PCB
 * (branch code_state_machine, board_obc/Core/Src/main.c). Tudo roda na task de housekeeping (único
 * usuário do I2C1 junto com BME280 e DS3231). */
#ifndef EPS_H
#define EPS_H

#include <stdint.h>
#include "stm32l4xx_hal.h"
#include "customTypes.h"

#define EPS_MUX_ADDR   0x70U
#define EPS_CH_BAT1    0U     /* MP2733 #1 (0x4B) + MAX17048 #1 (0x36) */
#define EPS_CH_BAT2    1U     /* MP2733 #2 (0x4B) + MAX17048 #2 (0x36) */
#define EPS_CH_INA1    2U     /* INA3221 0x40: tensões dos barramentos */
#define EPS_CH_INA2    3U     /* INA3221 0x41: correntes dos barramentos */

/* O MP2733 volta aos limites de fábrica se o watchdog dele expirar: o board_obc reaplica os limites a cada
 * ~1 s. Aqui o housekeeping acorda a cada EPS_KEEPALIVE_MS só para isso, além da leitura completa a cada 60 s. */
#define EPS_KEEPALIVE_MS 5000U

/* Bits de eps_presentes() */
#define EPS_TEM_MUX   (1U << 0)
#define EPS_TEM_MP1   (1U << 1)
#define EPS_TEM_MP2   (1U << 2)
#define EPS_TEM_MAX1  (1U << 3)
#define EPS_TEM_MAX2  (1U << 4)
#define EPS_TEM_INA1  (1U << 5)
#define EPS_TEM_INA2  (1U << 6)

/* Detecta e configura o que responder. Pode ser chamada de novo: tenta só o que faltou. */
void eps_init(I2C_HandleTypeDef *hi2c);
/* Lê tudo e preenche os campos de EPS do pacote (os que não responderem ficam com o último valor). */
void eps_ler(Telemetry_t *t);
/* Só reaplica os limites de corrente dos MP2733 (watchdog do carregador). */
void eps_keepalive(void);
uint8_t eps_presentes(void);

#endif

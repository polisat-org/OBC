/*
 * max17048.h
 *
 *  Created on: Sep 17, 2025
 *      Author: gabri
 */

#ifndef MAX17048_H
#define MAX17048_H

#include <stdint.h>
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_i2c.h"

/* Default 7-bit address is 0x36 -> HAL wants (0x36 << 1) */
#define MAX17048_I2C_ADDR_8BIT  (0x36 << 1)

/* Registers */
#define REG_VCELL    0x02
#define REG_SOC      0x04
#define REG_MODE     0x06
#define REG_VERSION  0x08
#define REG_CMD      0xFE

/* Timeouts */
#define MAX17048_I2C_TIMEOUT_MS  500

/* VCELL LSB: 78.125 uV per LSB (MAX17048 full-scale 5.12V / 65536). */
#define VCELL_LSB_V (78.125e-6f)


typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint16_t address; // HAL usa 8-bit address (7-bit << 1)
} MAX17048_HandleTypeDef;

/* Inicialização: passar o handle do I2C (ex: &hi2c1).
   Retorna HAL_OK se OK. */
HAL_StatusTypeDef MAX17048_Init(MAX17048_HandleTypeDef *dev, I2C_HandleTypeDef *hi2c);

/* Leitura/escrita genérica de registrador 16-bit (big-endian no chip) */
HAL_StatusTypeDef MAX17048_ReadReg(MAX17048_HandleTypeDef *dev, uint8_t reg, uint16_t *value);
HAL_StatusTypeDef MAX17048_WriteReg(MAX17048_HandleTypeDef *dev, uint8_t reg, uint16_t value);

/* Leituras úteis */
HAL_StatusTypeDef MAX17048_GetVoltage(MAX17048_HandleTypeDef *dev, float *voltage_v); // VCELL
HAL_StatusTypeDef MAX17048_GetSOC(MAX17048_HandleTypeDef *dev, float *soc_percent);    // SOC % (0.00 - 100.00)

/* Comandos especiais */
HAL_StatusTypeDef MAX17048_QuickStart(MAX17048_HandleTypeDef *dev); // escreve Quick-Start bit no MODE
HAL_StatusTypeDef MAX17048_ResetPOR(MAX17048_HandleTypeDef *dev);   // escreve POR comando no CMD (0xFE)

/* Leitura da versão (registro 0x08) */
HAL_StatusTypeDef MAX17048_ReadVersion(MAX17048_HandleTypeDef *dev, uint16_t *version);

#endif // MAX17048_H

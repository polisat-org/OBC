/*
 * max17048.c
 *
 *  Created on: Sep 17, 2025
 *      Author: gabri
 */

#include "max17048.h"
#include <string.h>

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

HAL_StatusTypeDef MAX17048_Init(MAX17048_HandleTypeDef *dev, I2C_HandleTypeDef *hi2c) {
    if (!dev || !hi2c) return HAL_ERROR; // Checa se ponteiros sao nulos; se sim, da erro
    dev->hi2c = hi2c; // Passa canal i2c
    dev->address = MAX17048_I2C_ADDR_8BIT; // Passa endereco do MAX
    return HAL_OK;
}

static HAL_StatusTypeDef i2c_write16(MAX17048_HandleTypeDef *dev, uint8_t reg, uint16_t value) {
    uint8_t buf[2]; // Coloca valor de escrita de 16 bits em vetor de 8 bits para transmissao i2c 
    buf[0] = (uint8_t)(value >> 8);
    buf[1] = (uint8_t)(value & 0xFF);
    return HAL_I2C_Mem_Write(dev->hi2c, dev->address, reg, I2C_MEMADD_SIZE_8BIT, buf, 2, MAX17048_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef i2c_read16(MAX17048_HandleTypeDef *dev, uint8_t reg, uint16_t *out) {
    uint8_t buf[2];
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(dev->hi2c, dev->address, reg, I2C_MEMADD_SIZE_8BIT, buf, 2, MAX17048_I2C_TIMEOUT_MS);
    if (st != HAL_OK) return st;
    *out = ((uint16_t)buf[0] << 8) | buf[1];
    return HAL_OK;
}

HAL_StatusTypeDef MAX17048_ReadReg(MAX17048_HandleTypeDef *dev, uint8_t reg, uint16_t *value) {
    return i2c_read16(dev, reg, value);
}

HAL_StatusTypeDef MAX17048_WriteReg(MAX17048_HandleTypeDef *dev, uint8_t reg, uint16_t value) {
    return i2c_write16(dev, reg, value);
}

HAL_StatusTypeDef MAX17048_GetVoltage(MAX17048_HandleTypeDef *dev, float *voltage_v) {
    uint16_t raw;
    HAL_StatusTypeDef st = i2c_read16(dev, REG_VCELL, &raw);
    if (st != HAL_OK) return st;
    /* raw is 16-bit value; convert using LSB */
    *voltage_v = (float)raw * VCELL_LSB_V;
    return HAL_OK;
}

HAL_StatusTypeDef MAX17048_GetSOC(MAX17048_HandleTypeDef *dev, float *soc_percent) {
    uint16_t raw; // O primeiro byte tem unidade de 1%, o segundo adiciona resolucao
    HAL_StatusTypeDef st = i2c_read16(dev, REG_SOC, &raw);
    if (st != HAL_OK) return st;
    uint8_t integer = (uint8_t)(raw >> 8); // Parte inteira da porcentagem
    uint8_t frac = (uint8_t)(raw & 0xFF); // Parte fracionada, com unidade 1/256 %
    *soc_percent = (float)integer + ((float)frac / 256.0f);
    return HAL_OK;
}

/* QuickStart: set Quick-Start bit in MODE register (datasheet: write 1 to quick-start bit) */
/* Implementation used by many drivers: write 0x4000 to MODE (bit14). */
HAL_StatusTypeDef MAX17048_QuickStart(MAX17048_HandleTypeDef *dev) {
    return i2c_write16(dev, REG_MODE, 0x4000);
}

/* POR command: write 0xFFFF to CMD (0xFE). Causes power-on reset behavior. */
HAL_StatusTypeDef MAX17048_ResetPOR(MAX17048_HandleTypeDef *dev) {
    return i2c_write16(dev, REG_CMD, 0xFFFF);
}

HAL_StatusTypeDef MAX17048_ReadVersion(MAX17048_HandleTypeDef *dev, uint16_t *version) {
    return i2c_read16(dev, REG_VERSION, version);
}


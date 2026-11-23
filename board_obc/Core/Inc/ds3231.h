/**
 ******************************************************************************
 * @file    ds3231.h
 * @brief   Driver minimalista para o RTC DS3231 via I2C com HAL STM32.
 *          Hora em 24 h, anos de 2000 a 2099, temperatura em 0.01 grau C.
 ******************************************************************************
 */
#ifndef DS3231_H
#define DS3231_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"   /* traz o HAL (stm32l4xx_hal.h) */
#include <stdint.h>

/* Endereco I2C fixo (0x68) ja deslocado, formato esperado pelo HAL */
#define DS3231_I2C_ADDR   (0x68U << 1)

typedef enum
{
  DS3231_OK = 0,
  DS3231_ERR_I2C,
  DS3231_ERR_PARAM
} DS3231_Status;

typedef struct
{
  uint8_t  sec;     /* 0..59 */
  uint8_t  min;     /* 0..59 */
  uint8_t  hour;    /* 0..23 */
  uint8_t  wday;    /* 1..7, 1 = segunda ... 7 = domingo (calculado por SetTime) */
  uint8_t  day;     /* 1..31 */
  uint8_t  month;   /* 1..12 */
  uint16_t year;    /* 2000..2099 */
} DS3231_Time_t;

typedef struct
{
  I2C_HandleTypeDef *hi2c;
} DS3231_t;

/** Verifica se o chip responde em 0x68 e garante que o oscilador roda com bateria. */
DS3231_Status DS3231_Init(DS3231_t *dev, I2C_HandleTypeDef *hi2c);

/** Le data e hora (burst de 7 registradores, sem leitura "rasgada"). */
DS3231_Status DS3231_GetTime(DS3231_t *dev, DS3231_Time_t *t);

/** Acerta data e hora. O dia da semana e calculado a partir da data (t->wday e ignorado).
 *  Tambem limpa o bit OSF (hora valida). */
DS3231_Status DS3231_SetTime(DS3231_t *dev, const DS3231_Time_t *t);

/** stopped = 1 se o bit OSF estiver setado: o oscilador parou e a hora nao e confiavel. */
DS3231_Status DS3231_OscStopped(DS3231_t *dev, uint8_t *stopped);

/** Temperatura interna em 0.01 grau C (2475 = 24.75 C). Resolucao de 0.25 C,
 *  atualizada pelo chip a cada 64 s. */
DS3231_Status DS3231_GetTemperature(DS3231_t *dev, int16_t *temp_centi);

/** Dia da semana para uma data: 1 = segunda ... 7 = domingo. */
uint8_t DS3231_CalcWeekday(uint16_t year, uint8_t month, uint8_t day);

/** Converte __DATE__ ("Nov  8 2026") e __TIME__ ("14:32:07") em DS3231_Time_t. */
DS3231_Status DS3231_FromCompileTime(DS3231_Time_t *t, const char *date, const char *time);

#ifdef __cplusplus
}
#endif

#endif /* DS3231_H */
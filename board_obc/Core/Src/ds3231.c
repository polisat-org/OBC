/**
 ******************************************************************************
 * @file    ds3231.c
 * @brief   Driver minimalista para o RTC DS3231 via I2C com HAL STM32.
 ******************************************************************************
 */
#include "ds3231.h"
#include <string.h>

/* ---------------------------- Registradores ------------------------------ */
#define REG_SECONDS      0x00U   /* 0x00..0x06: seg, min, hora, dia semana, dia, mes, ano */
#define REG_CONTROL      0x0EU
#define REG_STATUS       0x0FU
#define REG_TEMP_MSB     0x11U   /* 0x11 (parte inteira), 0x12 (bits 7:6 = 0.25 C) */

#define CTRL_EOSC        0x80U   /* 1 = oscilador desligado quando alimentado pela bateria */
#define STATUS_OSF       0x80U   /* 1 = o oscilador parou em algum momento */

#define I2C_TIMEOUT_MS   100U

#define CHK(x) do { DS3231_Status s_ = (x); if (s_ != DS3231_OK) return s_; } while (0)

/* ------------------------------ Utilitarios ------------------------------ */
static uint8_t bcd2bin(uint8_t v)
{
  return (uint8_t)(((v >> 4) * 10U) + (v & 0x0FU));
}

static uint8_t bin2bcd(uint8_t v)
{
  return (uint8_t)(((v / 10U) << 4) | (v % 10U));
}

static DS3231_Status rd(DS3231_t *d, uint8_t reg, uint8_t *buf, uint16_t len)
{
  return (HAL_I2C_Mem_Read(d->hi2c, DS3231_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                           buf, len, I2C_TIMEOUT_MS) == HAL_OK) ? DS3231_OK : DS3231_ERR_I2C;
}

static DS3231_Status wr(DS3231_t *d, uint8_t reg, const uint8_t *buf, uint16_t len)
{
  return (HAL_I2C_Mem_Write(d->hi2c, DS3231_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                            (uint8_t *)buf, len, I2C_TIMEOUT_MS) == HAL_OK) ? DS3231_OK : DS3231_ERR_I2C;
}

/* ------------------------------ API publica ------------------------------ */
DS3231_Status DS3231_Init(DS3231_t *d, I2C_HandleTypeDef *hi2c)
{
  uint8_t ctrl = 0;

  d->hi2c = hi2c;

  if (HAL_I2C_IsDeviceReady(hi2c, DS3231_I2C_ADDR, 2, 10) != HAL_OK)
  {
    return DS3231_ERR_I2C;
  }

  /* EOSC = 0 garante que o relogio continua contando quando so a bateria alimenta */
  CHK(rd(d, REG_CONTROL, &ctrl, 1));
  if (ctrl & CTRL_EOSC)
  {
    ctrl = (uint8_t)(ctrl & ~CTRL_EOSC);
    CHK(wr(d, REG_CONTROL, &ctrl, 1));
  }
  return DS3231_OK;
}

DS3231_Status DS3231_GetTime(DS3231_t *d, DS3231_Time_t *t)
{
  uint8_t r[7];
  uint8_t h;

  CHK(rd(d, REG_SECONDS, r, sizeof(r)));

  t->sec = bcd2bin(r[0] & 0x7FU);
  t->min = bcd2bin(r[1] & 0x7FU);

  h = r[2];
  if (h & 0x40U)
  {
    /* Chip em modo 12 h (nao e o padrao deste driver): converte para 24 h */
    uint8_t hh = bcd2bin(h & 0x1FU);
    if (h & 0x20U)            /* PM */
    {
      if (hh != 12U) { hh = (uint8_t)(hh + 12U); }
    }
    else if (hh == 12U)       /* 12 AM = 00 h */
    {
      hh = 0U;
    }
    t->hour = hh;
  }
  else
  {
    t->hour = bcd2bin(h & 0x3FU);
  }

  t->wday  = (uint8_t)(r[3] & 0x07U);
  t->day   = bcd2bin(r[4] & 0x3FU);
  t->month = bcd2bin(r[5] & 0x1FU);            /* bit 7 = seculo, ignorado */
  t->year  = (uint16_t)(2000U + bcd2bin(r[6]));
  return DS3231_OK;
}

DS3231_Status DS3231_SetTime(DS3231_t *d, const DS3231_Time_t *t)
{
  uint8_t r[7];
  uint8_t st = 0;

  if ((t->sec > 59U) || (t->min > 59U) || (t->hour > 23U) ||
      (t->day < 1U) || (t->day > 31U) || (t->month < 1U) || (t->month > 12U) ||
      (t->year < 2000U) || (t->year > 2099U))
  {
    return DS3231_ERR_PARAM;
  }

  r[0] = bin2bcd(t->sec);
  r[1] = bin2bcd(t->min);
  r[2] = bin2bcd(t->hour);                       /* bit 6 = 0 -> modo 24 h */
  r[3] = DS3231_CalcWeekday(t->year, t->month, t->day);
  r[4] = bin2bcd(t->day);
  r[5] = bin2bcd(t->month);
  r[6] = bin2bcd((uint8_t)(t->year - 2000U));
  CHK(wr(d, REG_SECONDS, r, sizeof(r)));

  /* Hora valida de novo: limpa o flag OSF */
  CHK(rd(d, REG_STATUS, &st, 1));
  if (st & STATUS_OSF)
  {
    st = (uint8_t)(st & ~STATUS_OSF);
    CHK(wr(d, REG_STATUS, &st, 1));
  }
  return DS3231_OK;
}

DS3231_Status DS3231_OscStopped(DS3231_t *d, uint8_t *stopped)
{
  uint8_t st = 0;

  CHK(rd(d, REG_STATUS, &st, 1));
  *stopped = (st & STATUS_OSF) ? 1U : 0U;
  return DS3231_OK;
}

DS3231_Status DS3231_GetTemperature(DS3231_t *d, int16_t *temp_centi)
{
  uint8_t r[2];
  int16_t v;

  CHK(rd(d, REG_TEMP_MSB, r, sizeof(r)));

  /* 10 bits em complemento de dois: r[0] = inteiro, r[1] bits 7:6 = fracao */
  v = (int16_t)(((uint16_t)r[0] << 8) | r[1]);
  v = (int16_t)(v >> 6);                         /* passos de 0.25 C */
  *temp_centi = (int16_t)(v * 25);
  return DS3231_OK;
}

uint8_t DS3231_CalcWeekday(uint16_t year, uint8_t month, uint8_t day)
{
  /* Algoritmo de Sakamoto: 0 = domingo ... 6 = sabado */
  static const uint8_t tab[12] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
  uint32_t y = year;
  uint32_t dow;

  if (month < 3U)
  {
    y -= 1U;
  }
  dow = (y + (y / 4U) - (y / 100U) + (y / 400U) + tab[(month - 1U) % 12U] + day) % 7U;
  return (dow == 0U) ? 7U : (uint8_t)dow;        /* 1 = segunda ... 7 = domingo */
}

DS3231_Status DS3231_FromCompileTime(DS3231_Time_t *t, const char *date, const char *time)
{
  static const char months[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
  char mon[4];
  const char *p;

  if ((strlen(date) < 11U) || (strlen(time) < 8U))
  {
    return DS3231_ERR_PARAM;
  }

  mon[0] = date[0]; mon[1] = date[1]; mon[2] = date[2]; mon[3] = '\0';
  p = strstr(months, mon);
  if (p == NULL)
  {
    return DS3231_ERR_PARAM;
  }
  t->month = (uint8_t)(((uint32_t)(p - months) / 3U) + 1U);

  /* __DATE__ = "Mmm dd yyyy" (dia com espaco a esquerda se < 10) */
  t->day  = (uint8_t)(((date[4] == ' ') ? 0 : (date[4] - '0')) * 10 + (date[5] - '0'));
  t->year = (uint16_t)((date[7] - '0') * 1000 + (date[8] - '0') * 100 +
                       (date[9] - '0') * 10 + (date[10] - '0'));

  /* __TIME__ = "hh:mm:ss" */
  t->hour = (uint8_t)((time[0] - '0') * 10 + (time[1] - '0'));
  t->min  = (uint8_t)((time[3] - '0') * 10 + (time[4] - '0'));
  t->sec  = (uint8_t)((time[6] - '0') * 10 + (time[7] - '0'));

  t->wday = DS3231_CalcWeekday(t->year, t->month, t->day);
  return DS3231_OK;
}
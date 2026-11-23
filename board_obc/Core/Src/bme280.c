/**
 ******************************************************************************
 * @file    bme280.c
 * @brief   Driver minimalista para BME280 (e BMP280) via I2C com HAL STM32.
 ******************************************************************************
 */
#include "bme280.h"

/* ---------------------------- Registradores ------------------------------ */
#define REG_CALIB_00     0x88U   /* 0x88..0xA1 (26 bytes) */
#define REG_ID           0xD0U
#define REG_RESET        0xE0U
#define REG_CALIB_26     0xE1U   /* 0xE1..0xE7 (7 bytes)  */
#define REG_CTRL_HUM     0xF2U
#define REG_STATUS       0xF3U
#define REG_CTRL_MEAS    0xF4U
#define REG_CONFIG       0xF5U
#define REG_DATA         0xF7U   /* press[3] temp[3] hum[2] */

#define CMD_SOFT_RESET   0xB6U
#define STATUS_MEASURING 0x08U
#define STATUS_IM_UPDATE 0x01U

/* Oversampling: 1 = x1, 2 = x2, 3 = x4, 4 = x8, 5 = x16 */
#define OSRS_T           1U
#define OSRS_P           1U
#define OSRS_H           1U

#define I2C_TIMEOUT_MS   100U

#define CHK(x) do { BME280_Status s_ = (x); if (s_ != BME280_OK) return s_; } while (0)

/* ------------------------------ Baixo nivel ------------------------------ */
static BME280_Status rd(BME280_t *d, uint8_t reg, uint8_t *buf, uint16_t len)
{
  return (HAL_I2C_Mem_Read(d->hi2c, d->addr, reg, I2C_MEMADD_SIZE_8BIT,
                           buf, len, I2C_TIMEOUT_MS) == HAL_OK) ? BME280_OK : BME280_ERR_I2C;
}

static BME280_Status wr(BME280_t *d, uint8_t reg, uint8_t val)
{
  return (HAL_I2C_Mem_Write(d->hi2c, d->addr, reg, I2C_MEMADD_SIZE_8BIT,
                            &val, 1, I2C_TIMEOUT_MS) == HAL_OK) ? BME280_OK : BME280_ERR_I2C;
}

#define U16LE(b, i)  ((uint16_t)(((uint16_t)(b)[(i) + 1] << 8) | (b)[(i)]))
#define S16LE(b, i)  ((int16_t)U16LE(b, i))

/* ----------------------------- Calibracao -------------------------------- */
static BME280_Status read_calib(BME280_t *d)
{
  uint8_t b[26];
  uint8_t h[7];
  BME280_Calib_t *c = &d->calib;

  CHK(rd(d, REG_CALIB_00, b, sizeof(b)));

  c->T1 = U16LE(b, 0);  c->T2 = S16LE(b, 2);  c->T3 = S16LE(b, 4);
  c->P1 = U16LE(b, 6);  c->P2 = S16LE(b, 8);  c->P3 = S16LE(b, 10);
  c->P4 = S16LE(b, 12); c->P5 = S16LE(b, 14); c->P6 = S16LE(b, 16);
  c->P7 = S16LE(b, 18); c->P8 = S16LE(b, 20); c->P9 = S16LE(b, 22);
  c->H1 = b[25];        /* 0xA1 */

  if (d->chip_id == BME280_CHIP_ID_BME280)
  {
    CHK(rd(d, REG_CALIB_26, h, sizeof(h)));
    c->H2 = S16LE(h, 0);
    c->H3 = h[2];
    c->H4 = (int16_t)(((int8_t)h[3]) * 16 + (h[4] & 0x0F));
    c->H5 = (int16_t)(((int8_t)h[5]) * 16 + (h[4] >> 4));
    c->H6 = (int8_t)h[6];
  }
  return BME280_OK;
}

/* ---------------------- Compensacao (datasheet Bosch) -------------------- */
/* Temperatura em 0.01 grau C. Atualiza t_fine. */
static int32_t comp_temp(BME280_t *d, int32_t adc_T)
{
  const BME280_Calib_t *c = &d->calib;
  int32_t var1, var2;

  var1 = ((((adc_T >> 3) - ((int32_t)c->T1 << 1))) * ((int32_t)c->T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)c->T1)) *
            ((adc_T >> 4) - ((int32_t)c->T1))) >> 12) * ((int32_t)c->T3)) >> 14;
  d->t_fine = var1 + var2;
  return (d->t_fine * 5 + 128) >> 8;
}

/* Pressao em Pa, formato Q24.8 (divida por 256). */
static uint32_t comp_press_q24_8(const BME280_t *d, int32_t adc_P)
{
  const BME280_Calib_t *c = &d->calib;
  int64_t var1, var2, p;

  var1 = ((int64_t)d->t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)c->P6;
  var2 = var2 + ((var1 * (int64_t)c->P5) << 17);
  var2 = var2 + (((int64_t)c->P4) << 35);
  var1 = ((var1 * var1 * (int64_t)c->P3) >> 8) + ((var1 * (int64_t)c->P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)c->P1) >> 33;
  if (var1 == 0)
  {
    return 0;   /* evita divisao por zero */
  }
  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)c->P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)c->P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)c->P7) << 4);
  return (uint32_t)p;
}

/* Umidade em %RH, formato Q22.10 (divida por 1024). */
static uint32_t comp_hum_q22_10(const BME280_t *d, int32_t adc_H)
{
  const BME280_Calib_t *c = &d->calib;
  int32_t v;

  v = d->t_fine - ((int32_t)76800);
  v = (((((adc_H << 14) - (((int32_t)c->H4) << 20) - (((int32_t)c->H5) * v)) +
         ((int32_t)16384)) >> 15) *
       (((((((v * ((int32_t)c->H6)) >> 10) *
            (((v * ((int32_t)c->H3)) >> 11) + ((int32_t)32768))) >> 10) +
          ((int32_t)2097152)) * ((int32_t)c->H2) + 8192) >> 14));
  v = (v - (((((v >> 15) * (v >> 15)) >> 7) * ((int32_t)c->H1)) >> 4));
  v = (v < 0 ? 0 : v);
  v = (v > 419430400 ? 419430400 : v);
  return (uint32_t)(v >> 12);
}

/* ------------------------------ API publica ------------------------------ */
BME280_Status BME280_Init(BME280_t *d, I2C_HandleTypeDef *hi2c, uint16_t addr)
{
  uint8_t id = 0, st = 0;
  uint32_t t0;

  d->hi2c    = hi2c;
  d->addr    = addr;
  d->t_fine  = 0;
  d->chip_id = 0;

  CHK(rd(d, REG_ID, &id, 1));
  d->chip_id = id;   /* guardado mesmo se for inesperado (aparece no diagnostico) */
  if ((id != BME280_CHIP_ID_BME280) && (id != BME280_CHIP_ID_BMP280))
  {
    return BME280_ERR_ID;
  }

  /* Soft reset e espera a NVM ser copiada (bit im_update = 0) */
  CHK(wr(d, REG_RESET, CMD_SOFT_RESET));
  HAL_Delay(5);
  t0 = HAL_GetTick();
  do
  {
    CHK(rd(d, REG_STATUS, &st, 1));
  } while ((st & STATUS_IM_UPDATE) && ((HAL_GetTick() - t0) < 100U));

  CHK(read_calib(d));

  /* Filtro IIR desligado, sem standby (usado so em modo normal) */
  CHK(wr(d, REG_CONFIG, 0x00));
  return BME280_OK;
}

BME280_Status BME280_ReadForced(BME280_t *d, BME280_Data_t *out)
{
  uint8_t raw[8] = {0};
  uint8_t st = 0;
  uint32_t t0;
  const uint8_t has_hum = (d->chip_id == BME280_CHIP_ID_BME280);
  int32_t adc_P, adc_T, adc_H;

  /* ctrl_hum so tem efeito depois de escrever ctrl_meas */
  if (has_hum)
  {
    CHK(wr(d, REG_CTRL_HUM, OSRS_H));
  }
  CHK(wr(d, REG_CTRL_MEAS, (uint8_t)((OSRS_T << 5) | (OSRS_P << 2) | 0x01U)));

  /* Tempo maximo de medicao com x1/x1/x1 ~ 9.3 ms */
  HAL_Delay(2);
  t0 = HAL_GetTick();
  do
  {
    CHK(rd(d, REG_STATUS, &st, 1));
    if ((st & STATUS_MEASURING) == 0U)
    {
      break;
    }
  } while ((HAL_GetTick() - t0) < 50U);
  if (st & STATUS_MEASURING)
  {
    return BME280_ERR_TIMEOUT;
  }

  CHK(rd(d, REG_DATA, raw, has_hum ? 8 : 6));

  adc_P = (int32_t)(((uint32_t)raw[0] << 12) | ((uint32_t)raw[1] << 4) | (raw[2] >> 4));
  adc_T = (int32_t)(((uint32_t)raw[3] << 12) | ((uint32_t)raw[4] << 4) | (raw[5] >> 4));
  adc_H = has_hum ? (int32_t)(((uint32_t)raw[6] << 8) | raw[7]) : 0;

  /* A temperatura vem primeiro: ela define t_fine usado nas outras */
  out->temp_centi = comp_temp(d, adc_T);
  out->press_pa   = comp_press_q24_8(d, adc_P) / 256U;
  out->hum_centi  = has_hum ? (uint32_t)((comp_hum_q22_10(d, adc_H) * 100UL) / 1024UL) : 0U;

  return BME280_OK;
}
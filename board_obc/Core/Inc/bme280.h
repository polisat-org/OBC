  /**
   ******************************************************************************
  * @file    bme280.h
  * @brief   Driver minimalista para BME280 (e BMP280) via I2C com HAL STM32.
  *          Compensacao em inteiros (formulas do datasheet da Bosch).
  ******************************************************************************
  */
  #ifndef BME280_H
  #define BME280_H

  #ifdef __cplusplus
  extern "C" {
  #endif

  #include "main.h"   /* traz o HAL (stm32l4xx_hal.h) */
  #include <stdint.h>

  /* Enderecos I2C ja deslocados (formato esperado pelo HAL) */
  #define BME280_I2C_ADDR_LOW    (0x76U << 1)   /* SDO -> GND */
  #define BME280_I2C_ADDR_HIGH   (0x77U << 1)   /* SDO -> VDDIO */

  #define BME280_CHIP_ID_BME280  0x60U
  #define BME280_CHIP_ID_BMP280  0x58U          /* sem sensor de umidade */

  typedef enum
  {
    BME280_OK = 0,
    BME280_ERR_I2C,
    BME280_ERR_ID,
    BME280_ERR_TIMEOUT
  } BME280_Status;

  typedef struct
  {
    uint16_t T1; int16_t T2, T3;
    uint16_t P1; int16_t P2, P3, P4, P5, P6, P7, P8, P9;
    uint8_t  H1; int16_t H2; uint8_t H3; int16_t H4, H5; int8_t H6;
  } BME280_Calib_t;

  typedef struct
  {
    I2C_HandleTypeDef *hi2c;
    uint16_t           addr;      /* endereco de 7 bits ja deslocado */
    uint8_t            chip_id;
    int32_t            t_fine;
    BME280_Calib_t     calib;
  } BME280_t;

  typedef struct
  {
    int32_t  temp_centi;   /* 0.01 grau C  (2534 = 25.34 C) */
    uint32_t press_pa;     /* Pa           (101325 = 1013.25 hPa) */
    uint32_t hum_centi;    /* 0.01 %RH     (4633 = 46.33 %) ; 0 se BMP280 */
  } BME280_Data_t;

  /** Verifica ID, faz reset, le calibracao e configura o filtro. */
  BME280_Status BME280_Init(BME280_t *dev, I2C_HandleTypeDef *hi2c, uint16_t addr);

  /** Dispara uma medicao em modo forced (oversampling x1) e le o resultado. */
  BME280_Status BME280_ReadForced(BME280_t *dev, BME280_Data_t *out);


  
  #ifdef __cplusplus
  }
  #endif

  #endif /* BME280_H */
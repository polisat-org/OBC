#ifndef __BME280_H__
#define __BME280_H__

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

// AVISO!!! Essa biblioteca não é para ser usada com o BMP, mas sim o BME

/*
 * Datasheet usado:
 * https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf
 */

/*
 * chip_id's possíveis, essa biblioteca só funciona com o BME280, não com o BMP280,
 * se o seu chip é o BMP280, o sensor não vai ser inicializado!
 */

#define CHIP_ID_BME280			(0x60)

/*
 * Registradores de calibração, tabela 16 do datasheet
 */

#define CAL_REG_DIG_1			(0x88)	// (0x88) dig_T1 ~ dig_P9 (0x9F)
#define CAL_REG_DIG_2			(0xA1)	// (0xA1) dig_H1
#define CAL_REG_DIG_3			(0xE1)	// (0xE1) dig_H2 ~ dig_H6 (0xE7)

/*
 * Registradores de dados, tabela 17 do datasheet
 * São 8 registradores em sequência!
 */

#define DATA_REG				(0xF7)	// (0xF7) press_msb ~ hum_lsb (0xFE)

/*
 * Registradores de configuração e controle, tabela 17 do datasheet
 * Words de cada registrador
 * Default de cada registrador
 */

#define CFG_REG					(0xF5)	// t_sb[7:5] | filter[4:2] | X[1] | spi3w_en[0]
#define CFG_REG_WRD_STANDBY		(0xE0)	// 111000X0
#define CFG_REG_WRD_FILTER		(0x1C)	// 000111X0
#define CFG_REG_WRD_SPI_ON		(0x01)	// 000000X1
#define CFG_REG_DEFAULT			(0x40)	// 01000000 (125ms | filter off | X | spi off)

#define CTRL_REG_1				(0xF4)	// osrs_t[7:5] | osrs_p[4:2] | mode[1:0]
#define CTRL_REG_1_WRD_TEMP_OS	(0xE0)	// 11100000
#define CTRL_REG_1_WRD_PSSR_OS	(0x1C)	// 00011100
#define CTRL_REG_1_WRD_MODE		(0x03)	// 00000011
#define CTRL_REG_1_DEFAULT		(0x27)	// 00100111 (1x | 1x | normal mode)

#define CTRL_REG_2				(0xF2)	// XXXXX[7:3] | osrs_h[2:0]
#define CTRL_REG_2_WRD_HMDT_OS	(0x07)	// XXXXX111
#define CTRL_REG_2_DEFAULT		(0x01)	// 00000001 (X | 1x)

/*
 * Registrador de status, reset e chip_id, tabela 17 do datasheet
 * Words de cada registrador
 */

#define STTS_REG				(0xF3)	// X[7:4] | measuring[3] | X[2:1 | im_update[0]
#define STTS_REG_WRD_MEASURING	(0x08)	// XXXX1XX0
#define STTS_REG_WRD_IM_UPDATE	(0x01)	// XXXX0XX1

#define RESET_REG				(0xE0)	// reset[7:0]
#define RESET_VALUE				(0xB6)	// word que, quando escrita em RESET_REG, realiza um soft reset no sensor

#define ID_REG					(0xD0)	// chip_id[7:0]

/*
 * Typedef's para facilitar o trabalho
 * com o handle (confia, fica mais fácil)
 */

/*
 * Possíveis endereços, capítulo 6.2 do datasheet
 */

typedef enum {
	BME280_ADDRESS_GND = 0x76,
	BME280_ADDRESS_VIN = 0x77
} BME280_I2C_ADDRESS;

typedef enum {
	ALL = 0,
	TEMP = 1,
	PSSR = 2,
	HMDT = 3
} BME280_ARG;

typedef enum {					// Modos do sensor
    BME280_MODE_SLEEP  = 0x00,	// Sleep: não faz medições
    BME280_MODE_FORCED = 0x01,	// Forced: faz uma sequência de medições, segundo o oversampling configurado
    BME280_MODE_NORMAL = 0x03	// Normal: continuamente realiza medições, segundo o oversampling configurado, com o standby configurado
} BME280_MODE;

typedef enum {					// Equivalente a averaging, ou seja, para cada medição, faz N medidas e tira a média delas
	BME280_OFF    = 0x0,		// Desliga a respectiva medição
	BME280_OS_1x  = 0x01,		//  1x OS
	BME280_OS_2x  = 0x02,		//  2x OS
	BME280_OS_4x  = 0x03,		//  4x OS
	BME280_OS_8x  = 0x04,		//  8x OS
	BME280_OS_16x = 0x05		// 16x OS
} BME280_OS;

typedef enum {					// Tempo de standby entre medições no modo normal
	BME280_STDBY_0   = 0x00,	//    0.5 ms
	BME280_STDBY_10  = 0x06,	//   10.0 ms
	BME280_STDBY_20  = 0x07,	//   20.0 ms
	BME280_STDBY_1x  = 0x01,	//   62.5 ms
	BME280_STDBY_2x  = 0x02,	//  125.0 ms
	BME280_STDBY_4x  = 0x03,	//  250.0 ms
	BME280_STDBY_8x  = 0x04,	//  500.0 ms
	BME280_STDBY_16x = 0x05,	// 1000.0 ms
} BME280_STDBY;

typedef enum {					// Coeficiente de filtragem entre medições. No modo forced, o filtro não afeta quase nada, a não ser que o OS esteja bem alto
	BME280_FILTER_OFF = 0x00,	// Capítulo 3.4.4 do datasheet, não é muito complicado
	BME280_FILTER_2   = 0x01,
	BME280_FILTER_4   = 0x02,
	BME280_FILTER_8   = 0x03,
	BME280_FILTER_16  = 0x04,
} BME280_FILTER;

typedef struct {				// Struct para as configurações definidas acima
	BME280_MODE MODE;
	BME280_OS TEMP_OVERSAMPLING;
	BME280_OS PSSR_OVERSAMPLING;
	BME280_OS HMDT_OVERSAMPLING;
	BME280_STDBY STDBY;
	BME280_FILTER FILTER;
} BME280_CONFIG_TypeDef;

typedef struct {				// Aqui vão as medições do sensor
	float TEMP;
	float PSSR;
	float HMDT;
} BME280_READ_TypeDef;

typedef struct {				// Handle do sensor
	// Constantes de calibração, únicas de cada sensor. Na inicialização, esses números são recuperados e são usados
	// na correção dos dados brutos
	uint16_t dig_T1;
	int16_t  dig_T2;
	int16_t  dig_T3;

	uint16_t dig_P1;
	int16_t  dig_P2;
	int16_t  dig_P3;
	int16_t  dig_P4;
	int16_t  dig_P5;
	int16_t  dig_P6;
	int16_t  dig_P7;
	int16_t  dig_P8;
	int16_t  dig_P9;

	uint8_t  dig_H1;
	int16_t  dig_H2;
	uint8_t  dig_H3;
	int16_t  dig_H4;
	int16_t  dig_H5;
	int8_t   dig_H6;

	// Handle I2C, precisa ser configurado
	I2C_HandleTypeDef I2C_h;
	// Inicializa o typedef do endereço
	BME280_I2C_ADDRESS ADDR;
	// Inicializa a struct de configurações
	BME280_CONFIG_TypeDef CONFIG;
	// Inicializa a struct de leituras
	BME280_READ_TypeDef READ;

} BME280_HandleTypeDef;

bool BME280_initDefault(BME280_HandleTypeDef *BME280, I2C_HandleTypeDef I2C_Handle);

void BME280_setOversampling(BME280_HandleTypeDef *BME280, BME280_OS os, BME280_ARG bme280_arg);

void BME280_setStandby(BME280_HandleTypeDef *BME280, BME280_STDBY stdby);

void BME280_setFilter(BME280_HandleTypeDef *BME280, BME280_FILTER filter);

void BME280_sleep(BME280_HandleTypeDef *BME280);

void BME280_setNormalMode(BME280_HandleTypeDef *BME280);

void BME280_readSingleShot(BME280_HandleTypeDef *BME280);

void BME280_read(BME280_HandleTypeDef *BME280);

#endif  // __BME280_H__

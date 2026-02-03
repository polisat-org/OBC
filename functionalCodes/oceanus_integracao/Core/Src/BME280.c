#include "BME280.h"
#include <stdbool.h>

// Escrever nos registradores (8bit)
static void BME280_wireWriteRegister(BME280_HandleTypeDef *BME280, uint8_t reg, uint8_t value) {
	uint8_t cmd_buff[2];
    cmd_buff[0] = reg;
	cmd_buff[1] = value;

	HAL_I2C_Master_Transmit(&BME280->I2C_h, (BME280->ADDR << 1),(uint8_t*)cmd_buff,2,1000);
}

// Ler dos registradores (n*8bit)
static void BME280_wireReadRegister(BME280_HandleTypeDef *BME280, uint8_t reg, uint8_t *rx_buff, size_t size) {
	HAL_I2C_Master_Transmit(&BME280->I2C_h, (BME280->ADDR << 1), &reg, 1, 1000);
	HAL_I2C_Master_Receive(&BME280->I2C_h, (BME280->ADDR << 1), rx_buff, size, 1000);
}

// Ler os dados de calibração do sensor
static void BME280_readCalData(BME280_HandleTypeDef *BME280) {
	uint8_t cmd_buff1[24];
	uint8_t cmd_buff2[7];

	BME280_wireReadRegister(BME280, CAL_REG_DIG_1, cmd_buff1, 24);

	BME280->dig_T1 = (cmd_buff1[1] << 8) | cmd_buff1[0];
	BME280->dig_T2 = (cmd_buff1[3] << 8) | cmd_buff1[2];
	BME280->dig_T3 = (cmd_buff1[5] << 8) | cmd_buff1[4];
	BME280->dig_P1 = (cmd_buff1[7] << 8) | cmd_buff1[6];
	BME280->dig_P2 = (cmd_buff1[9] << 8) | cmd_buff1[8];
	BME280->dig_P3 = (cmd_buff1[11] << 8) | cmd_buff1[10];
	BME280->dig_P4 = (cmd_buff1[13] << 8) | cmd_buff1[12];
	BME280->dig_P5 = (cmd_buff1[15] << 8) | cmd_buff1[14];
	BME280->dig_P6 = (cmd_buff1[17] << 8) | cmd_buff1[16];
	BME280->dig_P7 = (cmd_buff1[19] << 8) | cmd_buff1[18];
	BME280->dig_P8 = (cmd_buff1[21] << 8) | cmd_buff1[20];
	BME280->dig_P9 = (cmd_buff1[23] << 8) | cmd_buff1[22];

	BME280_wireReadRegister(BME280, CAL_REG_DIG_2,  &BME280->dig_H1, 1);

	BME280_wireReadRegister(BME280, CAL_REG_DIG_3, cmd_buff2, 7);

	BME280->dig_H2 = (cmd_buff2[1] << 8) | cmd_buff2[0];
	BME280->dig_H3 = cmd_buff2[2];
	BME280->dig_H4 = (cmd_buff2[3] << 4) | (cmd_buff2[4] & 0x0F);
	BME280->dig_H5 = (cmd_buff2[4] >> 4) | (cmd_buff2[5] << 4);
	BME280->dig_H6 = cmd_buff2[6];
}

// Inicializa o sensor com os parâmetros padrão, endereco 76, se o chip não for BME retorna false
bool BME280_initDefault(BME280_HandleTypeDef *BME280, I2C_HandleTypeDef I2C_Handle) {
	BME280->ADDR = BME280_ADDRESS_GND; //padrao, endereco 76
	BME280->I2C_h = I2C_Handle;

	uint8_t chip_id;
	BME280_wireReadRegister(BME280, ID_REG, &chip_id, 1);
	if (chip_id != CHIP_ID_BME280) return false;

	BME280_wireWriteRegister(BME280, RESET_REG, RESET_VALUE);
	HAL_Delay(20);

	BME280_readCalData(BME280);
	HAL_Delay(20);

	BME280_wireWriteRegister(BME280, CFG_REG, CFG_REG_DEFAULT);
	HAL_Delay(20);
	BME280_wireWriteRegister(BME280, CTRL_REG_2, CTRL_REG_2_DEFAULT);
	HAL_Delay(20);
	BME280_wireWriteRegister(BME280, CTRL_REG_1, CTRL_REG_1_DEFAULT);
	HAL_Delay(20);

	BME280->CONFIG.STDBY = BME280_STDBY_2x;
	BME280->CONFIG.FILTER = BME280_FILTER_OFF;
	BME280->CONFIG.MODE = BME280_MODE_SLEEP;
	BME280->CONFIG.TEMP_OVERSAMPLING = BME280_OS_1x;
	BME280->CONFIG.PSSR_OVERSAMPLING = BME280_OS_1x;
	BME280->CONFIG.HMDT_OVERSAMPLING = BME280_OS_1x;

	return true;
}

// Inicializa o sensor com o segundo endereço, 77
bool BME280_initOther(BME280_HandleTypeDef *BME280, I2C_HandleTypeDef I2C_Handle) {
	BME280->ADDR = BME280_ADDRESS_VIN; // pino soldado (sdo com vin), endereco 77
	BME280->I2C_h = I2C_Handle;

	uint8_t chip_id;
	BME280_wireReadRegister(BME280, ID_REG, &chip_id, 1);
	if (chip_id != CHIP_ID_BME280) return false;

	BME280_wireWriteRegister(BME280, RESET_REG, RESET_VALUE);
	HAL_Delay(20);

	BME280_readCalData(BME280);
	HAL_Delay(20);

	BME280_wireWriteRegister(BME280, CFG_REG, CFG_REG_DEFAULT);
	HAL_Delay(20);
	BME280_wireWriteRegister(BME280, CTRL_REG_2, CTRL_REG_2_DEFAULT);
	HAL_Delay(20);
	BME280_wireWriteRegister(BME280, CTRL_REG_1, CTRL_REG_1_DEFAULT);
	HAL_Delay(20);

	BME280->CONFIG.STDBY = BME280_STDBY_2x;
	BME280->CONFIG.FILTER = BME280_FILTER_OFF;
	BME280->CONFIG.MODE = BME280_MODE_SLEEP;
	BME280->CONFIG.TEMP_OVERSAMPLING = BME280_OS_1x;
	BME280->CONFIG.PSSR_OVERSAMPLING = BME280_OS_1x;
	BME280->CONFIG.HMDT_OVERSAMPLING = BME280_OS_1x;

	return true;
}

// Algoritmos de compensação
static int32_t BME280_compensateTemp(BME280_HandleTypeDef *BME280, int32_t adc_temp, int32_t *fine_temp) {
	int32_t var1, var2;

	var1 = ((((adc_temp >> 3) - ((int32_t) BME280->dig_T1 << 1)))
			* (int32_t) BME280->dig_T2) >> 11;
	var2 = (((((adc_temp >> 4) - (int32_t) BME280->dig_T1)
			* ((adc_temp >> 4) - (int32_t) BME280->dig_T1)) >> 12)
			* (int32_t) BME280->dig_T3) >> 14;

	*fine_temp = var1 + var2;
	return (*fine_temp * 5 + 128) >> 8;
}

static uint32_t BME280_compensatePssr(BME280_HandleTypeDef *BME280, int32_t adc_press, int32_t fine_temp) {
	int64_t var1, var2, p;

	var1 = (int64_t) fine_temp - 128000;
	var2 = var1 * var1 * (int64_t) BME280->dig_P6;
	var2 = var2 + ((var1 * (int64_t) BME280->dig_P5) << 17);
	var2 = var2 + (((int64_t) BME280->dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t) BME280->dig_P3) >> 8)
			+ ((var1 * (int64_t) BME280->dig_P2) << 12);
	var1 = (((int64_t) 1 << 47) + var1) * ((int64_t) BME280->dig_P1) >> 33;

	if (var1 == 0) {
		return 0;  // avoid exception caused by division by zero
	}

	p = 1048576 - adc_press;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = ((int64_t) BME280->dig_P9 * (p >> 13) * (p >> 13)) >> 25;
	var2 = ((int64_t) BME280->dig_P8 * p) >> 19;

	p = ((p + var1 + var2) >> 8) + ((int64_t) BME280->dig_P7 << 4);
	return p;
}

static uint32_t BME280_compensateHmdt(BME280_HandleTypeDef *BME280, int32_t adc_hum, int32_t fine_temp) {
	int32_t v_x1_u32r;

	v_x1_u32r = fine_temp - (int32_t) 76800;
	v_x1_u32r = ((((adc_hum << 14) - ((int32_t) BME280->dig_H4 << 20)
			- ((int32_t) BME280->dig_H5 * v_x1_u32r)) + (int32_t) 16384) >> 15)
			* (((((((v_x1_u32r * (int32_t) BME280->dig_H6) >> 10)
					* (((v_x1_u32r * (int32_t) BME280->dig_H3) >> 11)
							+ (int32_t) 32768)) >> 10) + (int32_t) 2097152)
					* (int32_t) BME280->dig_H2 + 8192) >> 14);
	v_x1_u32r = v_x1_u32r
			- (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7)
					* (int32_t) BME280->dig_H1) >> 4);
	v_x1_u32r = v_x1_u32r < 0 ? 0 : v_x1_u32r;
	v_x1_u32r = v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r;
	return v_x1_u32r >> 12;
}

// Configura o oversampling para a medição especificada em bme280_arg
void BME280_setOversampling(BME280_HandleTypeDef *BME280, BME280_OS os, BME280_ARG bme280_arg) {
	switch (bme280_arg) {
	case ALL:
		BME280->CONFIG.TEMP_OVERSAMPLING = os;
		BME280->CONFIG.PSSR_OVERSAMPLING = os;
		BME280->CONFIG.HMDT_OVERSAMPLING = os;
		break;
	case TEMP:
		BME280->CONFIG.TEMP_OVERSAMPLING = os;
		break;
	case PSSR:
		BME280->CONFIG.PSSR_OVERSAMPLING = os;
		break;
	case HMDT:
		BME280->CONFIG.HMDT_OVERSAMPLING = os;
		break;
	}

	if (bme280_arg == ALL || bme280_arg == TEMP || bme280_arg == PSSR) {
		BME280_wireWriteRegister(BME280, CTRL_REG_1, (BME280->CONFIG.TEMP_OVERSAMPLING << 5) | (BME280->CONFIG.PSSR_OVERSAMPLING << 2) | (BME280->CONFIG.MODE));
	}
	if (bme280_arg == ALL || bme280_arg == HMDT) {
		BME280_wireWriteRegister(BME280, CTRL_REG_2, os);
	}
}

// Configura o standby entre leituras para o modo normal
void BME280_setStandby(BME280_HandleTypeDef *BME280, BME280_STDBY stdby) {
	uint8_t rx_buff;
	BME280_wireReadRegister(BME280, CFG_REG, &rx_buff, 1);
	BME280_wireWriteRegister(BME280, CFG_REG, (rx_buff & 0x1F) | (stdby << 5));
	BME280->CONFIG.STDBY = stdby;
}

// Configura o filtro
void BME280_setFilter(BME280_HandleTypeDef *BME280, BME280_FILTER filter) {
	uint8_t rx_buff;
	BME280_wireReadRegister(BME280, CFG_REG, &rx_buff, 1);
	BME280_wireWriteRegister(BME280, CFG_REG, (rx_buff & 0x1F) | (filter << 2));
	BME280->CONFIG.FILTER = filter;
}

// Coloca o sensor no modo sleep
void BME280_sleep(BME280_HandleTypeDef *BME280) {
	uint8_t rx_buff;
	BME280_wireReadRegister(BME280, CTRL_REG_1, &rx_buff, 1);
	BME280_wireWriteRegister(BME280, CTRL_REG_1, (rx_buff & 0xFC) | BME280_MODE_SLEEP);
	BME280->CONFIG.MODE = BME280_MODE_SLEEP;
}

// Coloca o sensor no modo normal
void BME280_setNormalMode(BME280_HandleTypeDef *BME280) {
	uint8_t rx_buff;
	BME280_wireReadRegister(BME280, CTRL_REG_1, &rx_buff, 1);
	BME280_wireWriteRegister(BME280, CTRL_REG_1, (rx_buff & 0xFC) | BME280_MODE_NORMAL);
	BME280->CONFIG.MODE = BME280_MODE_NORMAL;
}

// Coloca o sensor no modo forced e faz uma leitura
void BME280_readSingleShot(BME280_HandleTypeDef *BME280) {
	uint8_t rx_buff;
	BME280_wireReadRegister(BME280, CTRL_REG_1, &rx_buff, 1);
	BME280_wireWriteRegister(BME280, CTRL_REG_1, (rx_buff & 0xFC) | BME280_MODE_FORCED);
	BME280->CONFIG.MODE = BME280_MODE_FORCED;
	HAL_Delay(20);
	BME280_read(BME280);
}

// Faz uma leitura dos registradores de dados medidos
void BME280_read(BME280_HandleTypeDef *BME280) {
	uint8_t cmd_buff[8];
	BME280_wireReadRegister(BME280, DATA_REG, cmd_buff, 8);

	int32_t temp, pssr, hmdt;
	temp = ((cmd_buff[3] << 12) | (cmd_buff[4] << 4) | (cmd_buff[5] >> 4));
	pssr = ((cmd_buff[0] << 12) | (cmd_buff[1] << 4) | (cmd_buff[2] >> 4));
	hmdt = ((cmd_buff[6] << 8) | (cmd_buff[7]));

	int32_t t_fine;
	BME280->READ.TEMP = (float) BME280_compensateTemp(BME280, temp, &t_fine)/100;
	BME280->READ.PSSR = (float) BME280_compensatePssr(BME280, pssr, t_fine)/256;
	BME280->READ.HMDT = (float) BME280_compensateHmdt(BME280, hmdt, t_fine)/1024;
}




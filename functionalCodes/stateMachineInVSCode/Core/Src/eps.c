/* EPS na telemetria (ver eps.h). Porte da leitura do board_obc (branch code_state_machine) sem as
 * mensagens de USB: mesmos drivers, endereços, canais do MUX, limites do MP2733 e mapeamento. */
#include "eps.h"
#include <string.h>
#include "ina3221.h"
#include "max17048.h"
#include "mp2733.h"

/* Limites do MP2733: os mesmos do keepalive da bancada (bancada/eps_nano/eps_nano.ino, MP_LIM), que é a
 * configuração validada. Escalas do datasheet MP2733 Rev. 1.1:
 *   REG00h: EN_LIM (bit 6) + IIN_LIM (bits 5-0) = 100 mA + n x 50 mA   -> 0x52 = EN_LIM ligado, 1000 mA
 *   REG05h: ICC (bits 6-0) = 320 mA + n x 40 mA                        -> 0x11 = 1000 mA
 *   REG01h: VIN_MIN (bits 6-0) = 3,7 V + n x 100 mV                    -> 0x00 = 3,7 V
 *   REG04h: VSYS_MIN (bits 3-1), BAT_LOADEN (bit 7)                    -> 110 = 3,675 V, BAT_LOADEN = 0
 *   REG02h: EN_CHG_NTC = 0 (sem termistor na placa)                    -> 0xC9 */
#define MP2733_REG_IINLIM    0x00U
#define MP2733_IINLIM_MASK   0x7FU
#define MP2733_IINLIM_1_0A   0x52U
#define MP2733_REG_ICHG      0x05U
#define MP2733_ICHG_MASK     0x7FU
#define MP2733_ICHG_1_0A     0x11U
#define MP2733_REG_VINMIN    0x01U
#define MP2733_VINMIN_MASK   0x7FU   /* bit 7 = REGISTER_RESET: não mexe */
#define MP2733_VINMIN_3_7V   0x00U
#define MP2733_REG_VSYS      0x04U
#define MP2733_VSYS_MASK     0x8EU   /* BAT_LOADEN + VSYS_MIN */
#define MP2733_VSYS_3_675V   0x0CU   /* BAT_LOADEN = 0, VSYS_MIN = 110 */
#define MP2733_REG_NTC       0x02U
#define MP2733_NTC_MASK      0xFFU
#define MP2733_NTC_SEM_TERM  0xC9U
#define MP2733_WD_RESET_BIT  0x08U   /* REG08h bit 3: reinicia o watchdog do MP2733 (volta a 0 sozinho) */

#define INA1_ADDR 0x40U
#define INA2_ADDR 0x41U

static I2C_HandleTypeDef *s_i2c;
static uint8_t s_tem;
static MP2733_HandleTypeDef s_mp[2];
static MAX17048_HandleTypeDef s_max[2];
static INA3221_Handle s_ina1, s_ina2;

static HAL_StatusTypeDef mux(uint8_t valor)
{
	return HAL_I2C_Master_Transmit(s_i2c, (uint16_t)(EPS_MUX_ADDR << 1), &valor, 1U, 100U);
}
static HAL_StatusTypeDef mux_canal(uint8_t ch) { return mux((uint8_t)(1U << ch)); }
static void mux_desliga(void) { (void)mux(0x00U); }

static void mp_limite(uint8_t reg, uint8_t mask, uint8_t valor)
{
	uint8_t v = 0;
	if (HAL_I2C_Mem_Read(s_i2c, MP2733_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, &v, 1U, 100U) == HAL_OK) {
		v = (uint8_t)((v & (uint8_t)~mask) | (valor & mask));
		(void)HAL_I2C_Mem_Write(s_i2c, MP2733_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, &v, 1U, 100U);
	}
}
/* ADC do MP2733 (datasheet Rev. 1.1): REG03h bit 6 ADC_RATE = 1 -> conversão contínua (o ADC_START,
 * bit 7, fica só-leitura); REG08h bit 6 EN_ADC_DSG = 1 -> ADC também sem entrada (bateria em standby).
 * Os dois voltam a 0 no watchdog do MP2733, por isso vão junto com os limites a cada keepalive. */
#define MP2733_ADC_RATE_BIT   0x40U
#define MP2733_EN_ADC_DSG_BIT 0x40U
#define MAX17048_REG_CRATE    0x16U   /* taxa de carga/descarga, int16, 0,208 %/h por LSB */

static void mp_limites(void)
{
	mp_limite(MP2733_REG_IINLIM, MP2733_IINLIM_MASK, MP2733_IINLIM_1_0A);
	mp_limite(MP2733_REG_ICHG, MP2733_ICHG_MASK, MP2733_ICHG_1_0A);
	mp_limite(MP2733_REG_VINMIN, MP2733_VINMIN_MASK, MP2733_VINMIN_3_7V);
	mp_limite(MP2733_REG_VSYS, MP2733_VSYS_MASK, MP2733_VSYS_3_675V);
	mp_limite(MP2733_REG_NTC, MP2733_NTC_MASK, MP2733_NTC_SEM_TERM);
	/* ADC em conversão contínua (só para a telemetria; não muda a carga) */
	mp_limite(MP2733_REG_ADC_OTG_CTRL, MP2733_ADC_RATE_BIT, MP2733_ADC_RATE_BIT);
	mp_limite(MP2733_REG_TIMER_CONFIG, MP2733_EN_ADC_DSG_BIT, MP2733_EN_ADC_DSG_BIT);
	/* Reinicia o watchdog do MP2733, como a bancada. Sem isso ele expira (40 s) e devolve ao padrão de fábrica
	 * os registradores "reset by WTD" (REG02..REG08, REG0A, REG16): NTC ligado (carga suspensa), ICC 1,84 A, ADC off. */
	mp_limite(MP2733_REG_TIMER_CONFIG, MP2733_WD_RESET_BIT, MP2733_WD_RESET_BIT);
}

/* Registradores de ADC do MP2733: 1 byte cada, escalas do datasheet (repetidas no solo):
 *   0x0F VSYS 20 mV/LSB · 0x10 NTC 0,392 %/LSB · 0x11 VIN 60 mV/LSB · 0x12 ICHG 17,5 mA/LSB · 0x13 IIN 13,3 mA/LSB */
typedef struct {
	uint8_t vsys, ntc, vin, ichg, iin;
	int16_t crate;
} EpsCarga_t;

static uint8_t mp_setup(MP2733_HandleTypeDef *h)
{
	uint8_t ok = MP2733_Init(h, s_i2c);
	HAL_Delay(50);
	if (HAL_I2C_IsDeviceReady(s_i2c, MP2733_ADDR << 1, 3, 100) != HAL_OK) return 0;
	if (!MP2733_GetStatus(h)) return 0;
	if (!MP2733_WriteConfig(h)) return 0;
	mp_limites();
	return ok ? 1U : 0U;
}

static uint8_t max_setup(MAX17048_HandleTypeDef *h)
{
	uint16_t versao = 0;
	if (HAL_I2C_IsDeviceReady(s_i2c, MAX17048_I2C_ADDR_8BIT, 3, 100) != HAL_OK) return 0;
	HAL_Delay(10);
	if (MAX17048_Init(h, s_i2c) != HAL_OK) return 0;
	return MAX17048_ReadVersion(h, &versao) == HAL_OK ? 1U : 0U;
}

static uint8_t ina_setup(INA3221_Handle *h, uint8_t addr)
{
	if (HAL_I2C_IsDeviceReady(s_i2c, (uint16_t)(addr << 1), 2, 50) != HAL_OK) return 0;
	INA3221_init(h, s_i2c, (INA3221_Addr)addr, 200U);
	INA3221_set_cfg(h, INA3221_CFG_RESET);
	HAL_Delay(10);
	return INA3221_default_config(h) == INA3221_OK ? 1U : 0U;
}

void eps_init(I2C_HandleTypeDef *hi2c)
{
	s_i2c = hi2c;
	if (!(s_tem & EPS_TEM_MUX)) {
		if (HAL_I2C_IsDeviceReady(s_i2c, (uint16_t)(EPS_MUX_ADDR << 1), 3, 50) != HAL_OK) return;
		s_tem |= EPS_TEM_MUX;
	}
	for (uint8_t b = 0; b < 2U; b++) {
		uint8_t bitMp = b ? EPS_TEM_MP2 : EPS_TEM_MP1, bitMax = b ? EPS_TEM_MAX2 : EPS_TEM_MAX1;
		if ((s_tem & bitMp) && (s_tem & bitMax)) continue;
		if (mux_canal(b ? EPS_CH_BAT2 : EPS_CH_BAT1) != HAL_OK) continue;
		if (!(s_tem & bitMp) && mp_setup(&s_mp[b])) s_tem |= bitMp;
		if (!(s_tem & bitMax) && max_setup(&s_max[b])) s_tem |= bitMax;
	}
	if (!(s_tem & EPS_TEM_INA1) && mux_canal(EPS_CH_INA1) == HAL_OK && ina_setup(&s_ina1, INA1_ADDR)) s_tem |= EPS_TEM_INA1;
	if (!(s_tem & EPS_TEM_INA2) && mux_canal(EPS_CH_INA2) == HAL_OK && ina_setup(&s_ina2, INA2_ADDR)) s_tem |= EPS_TEM_INA2;
	mux_desliga();   /* BME280 e DS3231 ficam no barramento principal */
}

void eps_keepalive(void)
{
	if (!(s_tem & EPS_TEM_MUX)) return;
	if ((s_tem & EPS_TEM_MP1) && mux_canal(EPS_CH_BAT1) == HAL_OK) mp_limites();
	if ((s_tem & EPS_TEM_MP2) && mux_canal(EPS_CH_BAT2) == HAL_OK) mp_limites();
	mux_desliga();
}

static void le_bateria(uint8_t b, uint8_t *status, uint8_t *falha, uint16_t *mv, uint16_t *soc, EpsCarga_t *c)
{
	if (s_tem & (b ? EPS_TEM_MP2 : EPS_TEM_MP1)) {
		mp_limites();
		/* Registradores crus: STATUS (0x0C) e FAULT (0x0D). O board_obc mandava só o retorno das funções
		 * (1 = leu), o que não diz nada no solo; o byte cru tem o estado de carga, a fonte e as falhas. */
		uint8_t r = 0;
		if (MP2733_ReadRegister(&s_mp[b], MP2733_REG_STATUS, &r)) *status = r;
		if (MP2733_ReadRegister(&s_mp[b], MP2733_REG_FAULT, &r)) *falha = r;
		if (MP2733_ReadRegister(&s_mp[b], MP2733_REG_ADC_SYS_VOLTAGE, &r)) c->vsys = r;
		if (MP2733_ReadRegister(&s_mp[b], MP2733_REG_ADC_THERMISTOR, &r)) c->ntc = r;
		if (MP2733_ReadRegister(&s_mp[b], MP2733_REG_ADC_INPUT_VOLTAGE, &r)) c->vin = r;
		if (MP2733_ReadRegister(&s_mp[b], MP2733_REG_ADC_CHARGE_CURRENT, &r)) c->ichg = r;
		if (MP2733_ReadRegister(&s_mp[b], MP2733_REG_ADC_INPUT_CURRENT, &r)) c->iin = r;
	}
	if (s_tem & (b ? EPS_TEM_MAX2 : EPS_TEM_MAX1)) {
		float v = 0.0f, s = 0.0f;
		if (MAX17048_GetSOC(&s_max[b], &s) == HAL_OK && MAX17048_GetVoltage(&s_max[b], &v) == HAL_OK) {
			int32_t m = (int32_t)(v * 1000.0f + 0.5f);
			if (m >= 0) *mv = (uint16_t)m;           /* mV */
			if (s >= 0.0f) *soc = (uint16_t)(s + 0.5f); /* % */
		}
		uint16_t cr = 0U;
		if (MAX17048_ReadReg(&s_max[b], MAX17048_REG_CRATE, &cr) == HAL_OK) c->crate = (int16_t)cr;
	}
}

void eps_ler(Telemetry_t *t)
{
	if (!(s_tem & EPS_TEM_MUX)) { eps_init(s_i2c); if (!(s_tem & EPS_TEM_MUX)) return; }
	/* Telemetry_t é packed: lê em variáveis locais e copia (nada de ponteiro para campo desalinhado). */
	uint8_t st[2] = {t->status_carregador_1, t->status_carregador_2};
	uint8_t fl[2] = {t->falha_carregador_1, t->falha_carregador_2};
	uint16_t mv[2] = {t->tensao_bateria_1, t->tensao_bateria_2};
	uint16_t soc[2] = {t->carga_bateria_1, t->carga_bateria_2};
	EpsCarga_t c[2] = {
		{ t->adc_tensao_sistema_1, t->adc_tensao_ntc_1, 0U, t->adc_corrente_carga_1, t->adc_corrente_sistema_1, t->taxa_carga_descarga_bateria_1 },
		{ t->adc_tensao_sistema_2, t->adc_tensao_ntc_2, 0U, t->adc_corrente_carga_2, t->adc_corrente_sistema_2, t->taxa_carga_descarga_bateria_2 },
	};
	c[0].vin = (uint8_t)(t->tensao_painel_solar_1 / 0.06f + 0.5f);
	c[1].vin = (uint8_t)(t->tensao_painel_solar_2 / 0.06f + 0.5f);
	if (mux_canal(EPS_CH_BAT1) == HAL_OK) le_bateria(0, &st[0], &fl[0], &mv[0], &soc[0], &c[0]);
	if (mux_canal(EPS_CH_BAT2) == HAL_OK) le_bateria(1, &st[1], &fl[1], &mv[1], &soc[1], &c[1]);
	/* Carga pelos dados do MP2733 (os INA3221 estão ligados errados na placa):
	 *   adc_tensao_sistema_n = VSYS (cru) · adc_tensao_ntc_n = NTC (cru) · adc_corrente_carga_n = ICHG (cru)
	 *   adc_corrente_sistema_n = IIN, corrente de ENTRADA do carregador (cru)
	 *   tensao_painel_solar_1/2 = VIN do carregador 1/2 em V (entrada: painel ou fonte); 3 e 4 ficam 0
	 *   taxa_carga_descarga_bateria_n = CRATE do MAX17048 (cru, 0,208 %/h; positivo = carregando) */
	for (uint8_t b = 0; b < 2U; b++) {
		uint8_t *vs = b ? &t->adc_tensao_sistema_2 : &t->adc_tensao_sistema_1;
		*vs = c[b].vsys;
		if (b) { t->adc_tensao_ntc_2 = c[b].ntc; t->adc_corrente_carga_2 = c[b].ichg; t->adc_corrente_sistema_2 = c[b].iin; t->taxa_carga_descarga_bateria_2 = c[b].crate; t->tensao_painel_solar_2 = (float)c[b].vin * 0.06f; }
		else   { t->adc_tensao_ntc_1 = c[b].ntc; t->adc_corrente_carga_1 = c[b].ichg; t->adc_corrente_sistema_1 = c[b].iin; t->taxa_carga_descarga_bateria_1 = c[b].crate; t->tensao_painel_solar_1 = (float)c[b].vin * 0.06f; }
	}
	t->status_carregador_1 = st[0]; t->status_carregador_2 = st[1];
	t->falha_carregador_1 = fl[0];  t->falha_carregador_2 = fl[1];
	t->tensao_bateria_1 = mv[0];    t->tensao_bateria_2 = mv[1];
	t->carga_bateria_1 = soc[0];    t->carga_bateria_2 = soc[1];

	if ((s_tem & EPS_TEM_INA1) && mux_canal(EPS_CH_INA1) == HAL_OK) {
		float v1 = 0.0f, v2 = 0.0f;
		if (INA3221_get_bus_voltage(&s_ina1, INA3221_CHAN_1, &v1) == INA3221_OK
				&& INA3221_get_bus_voltage(&s_ina1, INA3221_CHAN_2, &v2) == INA3221_OK) {
			t->tensao_13v8 = v1;   /* V; mapeamento do board_obc ("ajuste conforme os barramentos da placa") */
			t->tensao_5v = v2;
		}
	}
	if ((s_tem & EPS_TEM_INA2) && mux_canal(EPS_CH_INA2) == HAL_OK) {
		float i1 = 0.0f, i2 = 0.0f;
		if (INA3221_get_shunt_current(&s_ina2, INA3221_CHAN_1, &i1) == INA3221_OK
				&& INA3221_get_shunt_current(&s_ina2, INA3221_CHAN_2, &i2) == INA3221_OK) {
			t->corrente_13v8 = i1 / 1000.0f;   /* driver devolve mA; telemetria em A */
			t->corrente_5v = i2 / 1000.0f;
		}
	}
	mux_desliga();
	/* Tentativa de achar o que não respondeu no boot (bateria conectada depois, por exemplo). */
	if ((s_tem & 0x7FU) != 0x7FU) eps_init(s_i2c);
}

uint8_t eps_presentes(void) { return s_tem; }

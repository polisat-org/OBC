/* EPS na telemetria (ver eps.h). Porte da leitura do board_obc (branch code_state_machine) sem as
 * mensagens de USB: mesmos drivers, endereços, canais do MUX, limites do MP2733 e mapeamento. */
#include "eps.h"
#include <string.h>
#include "ina3221.h"
#include "max17048.h"
#include "mp2733.h"

/* Limites do MP2733 (iguais ao board_obc) */
#define MP2733_REG_ICHG      0x05U
#define MP2733_ICHG_MASK     0x7FU
#define MP2733_ICHG_1_0A     0x19U   /* 25 * 40 mA = 1000 mA */
#define MP2733_REG_IINLIM    0x00U
#define MP2733_IINLIM_MASK   0x7FU
#define MP2733_IINLIM_1_5A   0x1CU   /* 100 mA + 28 * 50 mA = 1500 mA */

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
static void mp_limites(void)
{
	mp_limite(MP2733_REG_IINLIM, MP2733_IINLIM_MASK, MP2733_IINLIM_1_5A);
	mp_limite(MP2733_REG_ICHG, MP2733_ICHG_MASK, MP2733_ICHG_1_0A);
}

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

static void le_bateria(uint8_t b, uint8_t *status, uint8_t *falha, uint16_t *mv, uint16_t *soc)
{
	if (s_tem & (b ? EPS_TEM_MP2 : EPS_TEM_MP1)) {
		mp_limites();
		/* Registradores crus: STATUS (0x0C) e FAULT (0x0D). O board_obc mandava só o retorno das funções
		 * (1 = leu), o que não diz nada no solo; o byte cru tem o estado de carga, a fonte e as falhas. */
		uint8_t r = 0;
		if (MP2733_ReadRegister(&s_mp[b], MP2733_REG_STATUS, &r)) *status = r;
		if (MP2733_ReadRegister(&s_mp[b], MP2733_REG_FAULT, &r)) *falha = r;
	}
	if (s_tem & (b ? EPS_TEM_MAX2 : EPS_TEM_MAX1)) {
		float v = 0.0f, s = 0.0f;
		if (MAX17048_GetSOC(&s_max[b], &s) == HAL_OK && MAX17048_GetVoltage(&s_max[b], &v) == HAL_OK) {
			int32_t m = (int32_t)(v * 1000.0f + 0.5f);
			if (m >= 0) *mv = (uint16_t)m;           /* mV */
			if (s >= 0.0f) *soc = (uint16_t)(s + 0.5f); /* % */
		}
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
	if (mux_canal(EPS_CH_BAT1) == HAL_OK) le_bateria(0, &st[0], &fl[0], &mv[0], &soc[0]);
	if (mux_canal(EPS_CH_BAT2) == HAL_OK) le_bateria(1, &st[1], &fl[1], &mv[1], &soc[1]);
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

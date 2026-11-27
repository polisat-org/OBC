/* Teste no PC da leitura de EPS (eps.c + drivers ina3221/max17048/mp2733) com um barramento I2C simulado:
 * MUX PCA9548A em 0x70 e, atrás dele, MP2733 + MAX17048 nos canais 0 e 1 e INA3221 (0x40, 0x41) nos canais 2 e 3.
 * Confere: valores na telemetria, MUX desligado ao fim (BME280/DS3231 no barramento principal),
 * reaplicação dos limites do MP2733 (watchdog) e detecção tardia de uma bateria. */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "eps.h"

#define CHECK(c) do { if (!(c)) { printf("FALHOU linha %d: %s\n", __LINE__, #c); return 1; } } while (0)

/* ------------------------------------------------------------------ barramento simulado */
enum { MP, MAX, INA };
typedef struct { int canal; uint8_t addr; int tipo; int presente; uint8_t r8[256]; uint16_t r16[256]; uint8_t ptr; } Dev;
static Dev devs[6];
static uint8_t muxMask, muxPresente = 1;   /* muxPresente = 0 simula placa sem EPS */
static int acessosComMuxAberto;   /* acesso a 0x68/0x76 com canal aberto (não deveria acontecer) */

static Dev *acha(uint16_t addr8) {
	uint8_t a = (uint8_t)(addr8 >> 1);
	Dev *achou = NULL; int n = 0;
	for (int i = 0; i < 6; i++)
		if (devs[i].presente && devs[i].addr == a && (muxMask & (1u << devs[i].canal))) { achou = &devs[i]; n++; }
	return n == 1 ? achou : NULL;   /* dois iguais com canais abertos = colisão = erro */
}
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *h, uint16_t addr, uint32_t t, uint32_t to) {
	(void)h; (void)t; (void)to;
	if ((addr >> 1) == 0x70) return muxPresente ? HAL_OK : HAL_ERROR;
	return acha(addr) ? HAL_OK : HAL_ERROR;
}
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *h, uint16_t addr, uint8_t *d, uint16_t n, uint32_t to) {
	(void)h; (void)to;
	if ((addr >> 1) == 0x70) { if (!muxPresente || n != 1) return HAL_ERROR; muxMask = d[0]; return HAL_OK; }
	Dev *v = acha(addr); if (!v) return HAL_ERROR;
	v->ptr = d[0];
	if (v->tipo == MP && n == 2) v->r8[d[0]] = d[1];
	if (v->tipo == INA && n == 3) v->r16[d[0]] = (uint16_t)(d[1] << 8 | d[2]);
	return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *h, uint16_t addr, uint8_t *d, uint16_t n, uint32_t to) {
	(void)h; (void)to;
	Dev *v = acha(addr); if (!v) return HAL_ERROR;
	if (v->tipo == MP) { for (int i = 0; i < n; i++) d[i] = v->r8[(uint8_t)(v->ptr + i)]; }
	else { uint16_t x = v->r16[v->ptr]; d[0] = (uint8_t)(x >> 8); if (n > 1) d[1] = (uint8_t)x; }
	return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *h, uint16_t addr, uint16_t reg, uint16_t rs, uint8_t *d, uint16_t n, uint32_t to) {
	(void)h; (void)rs; (void)to;
	if ((addr >> 1) == 0x68 || (addr >> 1) == 0x76) { if (muxMask) acessosComMuxAberto++; return HAL_OK; }
	Dev *v = acha(addr); if (!v) return HAL_ERROR;
	if (v->tipo == MP) { for (int i = 0; i < n; i++) d[i] = v->r8[(uint8_t)(reg + i)]; }
	else { uint16_t x = v->r16[reg]; d[0] = (uint8_t)(x >> 8); if (n > 1) d[1] = (uint8_t)x; }
	return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *h, uint16_t addr, uint16_t reg, uint16_t rs, uint8_t *d, uint16_t n, uint32_t to) {
	(void)h; (void)rs; (void)to;
	Dev *v = acha(addr); if (!v) return HAL_ERROR;
	if (v->tipo == MP) { for (int i = 0; i < n; i++) v->r8[(uint8_t)(reg + i)] = d[i]; }
	else v->r16[reg] = (uint16_t)(d[0] << 8 | (n > 1 ? d[1] : 0));
	return HAL_OK;
}
void HAL_Delay(uint32_t ms) { (void)ms; }

/* ------------------------------------------------------------------ teste */
static int perto(float a, float b) { return fabsf(a - b) < 1e-3f; }

int main(void) {
	I2C_HandleTypeDef i2c = {1};
	Dev *mp1 = &devs[0], *max1 = &devs[1], *mp2 = &devs[2], *max2 = &devs[3], *ina1 = &devs[4], *ina2 = &devs[5];
	*mp1 = (Dev){.canal = 0, .addr = 0x4B, .tipo = MP, .presente = 1};
	*max1 = (Dev){.canal = 0, .addr = 0x36, .tipo = MAX, .presente = 1};
	*mp2 = (Dev){.canal = 1, .addr = 0x4B, .tipo = MP, .presente = 1};
	*max2 = (Dev){.canal = 1, .addr = 0x36, .tipo = MAX, .presente = 0};   /* bateria 2 ligada depois */
	*ina1 = (Dev){.canal = 2, .addr = 0x40, .tipo = INA, .presente = 1};
	*ina2 = (Dev){.canal = 3, .addr = 0x41, .tipo = INA, .presente = 1};
	mp1->r8[0x0C] = 0x5B; mp1->r8[0x0D] = 0x00;
	mp2->r8[0x0C] = 0x48; mp2->r8[0x0D] = 0x80;
	/* ADC do MP2733 1 (1 byte por canal): VSYS 3,90 V, NTC 50,2 %, VIN 9,00 V, ICHG 700 mA, IIN 399 mA */
	mp1->r8[0x0F] = 195; mp1->r8[0x10] = 128; mp1->r8[0x11] = 150; mp1->r8[0x12] = 40; mp1->r8[0x13] = 30;
	max1->r16[0x16] = 0x0030;  /* CRATE +48 LSB = +9,98 %/h (carregando) */
	max1->r16[0x02] = 49920;   /* VCELL: 3,900 V (78,125 uV/LSB) */
	max1->r16[0x04] = 0x5A80;  /* SOC 90,5 % */
	max1->r16[0x08] = 0x0012;  /* VERSION */
	max2->r16[0x02] = 46080;   /* 3,600 V */
	max2->r16[0x04] = 0x2800;  /* 40 % */
	max2->r16[0x08] = 0x0012;
	ina1->r16[0x02] = (uint16_t)(1575 << 3);   /* bus CH1 12,600 V (8 mV/LSB) */
	ina1->r16[0x04] = (uint16_t)(625 << 3);    /* bus CH2 5,000 V */
	ina2->r16[0x01] = (uint16_t)(3300 << 3);   /* shunt CH1: 3300 * 0,4 mA = 1320 mA */
	ina2->r16[0x03] = (uint16_t)(2185 << 3);   /* shunt CH2: 874 mA */

	eps_init(&i2c);
	CHECK(muxMask == 0);
	CHECK(eps_presentes() == (EPS_TEM_MUX | EPS_TEM_MP1 | EPS_TEM_MP2 | EPS_TEM_MAX1 | EPS_TEM_INA1 | EPS_TEM_INA2));
	CHECK((mp1->r8[0x00] & 0x7F) == 0x1C && (mp1->r8[0x05] & 0x7F) == 0x19);   /* limites 1,5 A / 1,0 A */
	CHECK(ina1->r16[0x00] != 0 && ina2->r16[0x00] != 0);                        /* INA configurados */

	Telemetry_t t; memset(&t, 0, sizeof t);
	eps_ler(&t);
	CHECK(muxMask == 0);
	CHECK(t.status_carregador_1 == 0x5B && t.falha_carregador_1 == 0x00);
	CHECK(t.status_carregador_2 == 0x48 && t.falha_carregador_2 == 0x80);
	CHECK(t.tensao_bateria_1 == 3900 && t.carga_bateria_1 == 91);
	CHECK(t.adc_tensao_sistema_1 == 195 && t.adc_tensao_ntc_1 == 128 && t.adc_corrente_carga_1 == 40 && t.adc_corrente_sistema_1 == 30);
	CHECK(perto(t.tensao_painel_solar_1, 9.0f) && t.taxa_carga_descarga_bateria_1 == 48);
	CHECK((mp1->r8[0x03] & 0x40) && (mp1->r8[0x08] & 0x40));   /* ADC contínuo e EN_ADC_DSG ligados */
	CHECK(t.tensao_bateria_2 == 0 && t.carga_bateria_2 == 0);   /* bateria 2 ainda ausente */
	CHECK(perto(t.tensao_13v8, 12.6f) && perto(t.tensao_5v, 5.0f));
	CHECK(perto(t.corrente_13v8, 1.32f) && perto(t.corrente_5v, 0.874f));
	/* a leitura tentou achar de novo o que faltava: a bateria 2 aparece e entra na próxima leitura */
	max2->presente = 1;
	eps_ler(&t);
	CHECK(eps_presentes() & EPS_TEM_MAX2);
	eps_ler(&t);
	CHECK(t.tensao_bateria_2 == 3600 && t.carga_bateria_2 == 40);
	CHECK(t.tensao_bateria_1 == 3900);

	/* watchdog do MP2733 expirou: registradores voltam ao padrão; o keepalive reaplica os limites */
	mp1->r8[0x00] = 0x48; mp1->r8[0x05] = 0xA6; mp2->r8[0x00] = 0x48; mp2->r8[0x05] = 0xA6;
	eps_keepalive();
	CHECK(muxMask == 0);
	CHECK((mp1->r8[0x00] & 0x7F) == 0x1C && (mp1->r8[0x05] & 0x7F) == 0x19);
	CHECK((mp2->r8[0x00] & 0x7F) == 0x1C && (mp2->r8[0x05] & 0x7F) == 0x19);

	/* INA que para de responder: mantém o último valor e não trava */
	ina1->presente = 0;
	t.tensao_13v8 = 12.6f;
	eps_ler(&t);
	CHECK(perto(t.tensao_13v8, 12.6f) && muxMask == 0);

	CHECK(acessosComMuxAberto == 0);
	printf("EPS: todos os testes OK (MUX, 2 carregadores, 2 medidores, 2 INA3221, keepalive, bateria tardia)\n");
	return 0;
}

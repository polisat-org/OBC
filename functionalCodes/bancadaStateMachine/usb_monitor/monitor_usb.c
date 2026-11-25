/* Monitor pela USB nativa (ver monitor_usb.h). */
#include "monitor_usb.h"
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "cmsis_os.h"
#include "customTypes.h"
#include "usb_device.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

#define ESPERA_TERMINAL_MS 15000U   /* no boot, espera o terminal abrir (DTR) para não perder as primeiras linhas;
                                       terminal que não liga o DTR só atrasa o boot e funciona igual depois */
#define USB_OCUPADA_MS     50U

extern USBD_HandleTypeDef hUsbDeviceFS;
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern osMessageQueueId_t eventQueueHandle;

static volatile int s_ok, s_aberto, s_travado, s_pronto;
static osMutexId_t s_mtx;
static uint8_t s_buf[256];

void OTG_FS_IRQHandler(void) { HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS); }

/* 48 MHz para a USB sem mexer no SYSCLK (MSI 4 MHz): PLLSAI1 = 4 MHz x 24 / 2, com o MSI calibrado pelo cristal
 * de 32,768 kHz (modo MSI PLL). O board_obc fazia o mesmo com o MSI a 48 MHz direto. */
static int clock48(void)
{
	RCC_OscInitTypeDef o = {0};
	RCC_PeriphCLKInitTypeDef p = {0};
	__HAL_RCC_PWR_CLK_ENABLE();
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
	o.OscillatorType = RCC_OSCILLATORTYPE_LSE;
	o.LSEState = RCC_LSE_ON;
	o.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&o) != HAL_OK) return 0;
	HAL_RCCEx_EnableMSIPLLMode();
	p.PeriphClockSelection = RCC_PERIPHCLK_USB;
	p.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
	p.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
	p.PLLSAI1.PLLSAI1M = 1;
	p.PLLSAI1.PLLSAI1N = 24;
	p.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
	p.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
	p.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
	p.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
	return HAL_RCCEx_PeriphCLKConfig(&p) == HAL_OK;
}

int monitor_usb_init(void)
{
	/* a conta do PLLSAI1 supõe MSI a 4 MHz; sem o cristal LSE o clock da USB não fica preciso */
	if (HAL_RCC_GetSysClockFreq() != 4000000U || !clock48()) { s_pronto = 1; return 0; }
	s_mtx = osMutexNew(NULL);
	MX_USB_DEVICE_Init();
	s_ok = 1;
	for (uint32_t t = 0; t < ESPERA_TERMINAL_MS && !s_aberto; t += 100U) osDelay(100);
	osDelay(200);   /* o terminal costuma mandar a configuração da linha logo depois do DTR */
	s_pronto = 1;
	return 1;
}

void monitor_usb_espera(void)
{
	for (uint32_t t = 0; t < ESPERA_TERMINAL_MS + 2000U && !s_pronto; t += 50U) osDelay(50);
}

void monitor_usb_dtr(int aberto)
{
	s_aberto = aberto;
	s_travado = 0;
}

void monitor_usb_vprintf(const char *fmt, va_list ap)
{
	if (!s_ok || __get_IPSR() != 0U || hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) return;
	if (osMutexAcquire(s_mtx, 100U) != osOK) return;
	USBD_CDC_HandleTypeDef *h = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
	uint32_t t0 = osKernelGetTickCount();
	while (h != NULL && h->TxState != 0U) {
		/* ninguém lendo do outro lado: não espera de novo a cada linha, até a USB andar */
		if (s_travado || (osKernelGetTickCount() - t0) > USB_OCUPADA_MS) { s_travado = 1; goto fim; }
		osDelay(1);
	}
	s_travado = 0;
	if (h != NULL) {
		int n = vsnprintf((char *)s_buf, sizeof s_buf, fmt, ap);
		if (n > (int)sizeof s_buf - 1) n = (int)sizeof s_buf - 1;
		if (n > 0) (void)CDC_Transmit_FS(s_buf, (uint16_t)n);
	}
fim:
	osMutexRelease(s_mtx);
}

void monitor_usb_rx(const uint8_t *buf, uint32_t len)
{
	for (uint32_t i = 0; i < len; i++) {
		uint8_t c = buf[i], lc = (uint8_t)(c | 0x20U);
		int tc = (c >= '0' && c <= '9') ? (c - '0') : (lc >= 'a' && lc <= 'f') ? (lc - 'a' + 10) : -1;
		if (tc < 0) continue;   /* CR, LF, espaço */
		Event_t evt = { .type = EVT_RX_DONE, .timestamp = osKernelGetTickCount(), .data.telecommand = (uint8_t)tc };
		(void)osMessageQueuePut(eventQueueHandle, &evt, 0U, 0U);
	}
}

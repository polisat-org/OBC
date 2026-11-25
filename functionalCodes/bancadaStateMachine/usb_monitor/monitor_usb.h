/* Monitor pela USB nativa do STM32 (PA11/PA12, porta COM virtual), para a PCB do OBC, que não tem ST-Link.
 * Só entra na variante `build_flash.sh --monitor-usb` (-DMONITOR_USB=1); o firmware de voo não usa nada daqui.
 *   - saída: o debugPrintf do OBC vai para a porta COM virtual;
 *   - entrada: '0'..'9', 'a'..'f' digitados viram o telecomando 0x0..0xF, no mesmo caminho da USART1 (TT&C).
 * A pilha USB (ST USB Device Library + HAL PCD) é a mesma do board_obc. */
#ifndef MONITOR_USB_H
#define MONITOR_USB_H

#include <stdarg.h>
#include <stdint.h>

/* Liga o clock de 48 MHz e a USB. Chamar uma vez, com o FreeRTOS rodando (usa osDelay e mutex).
 * Devolve 0 se o cristal LSE não partiu (sem ele o clock da USB não fica preciso): o OBC segue sem monitor. */
int monitor_usb_init(void);

/* Para as outras tasks: espera o monitor ficar pronto (terminal aberto ou desistência), para as linhas do boot
 * (ex.: "eps presentes") não se perderem antes de alguém abrir a porta. */
void monitor_usb_espera(void);

/* Texto para a porta COM virtual. Descarta se a USB não estiver configurada ou se ficar ocupada > 50 ms
 * (porta fechada no PC); depois disso não espera de novo até a USB voltar a andar. */
void monitor_usb_vprintf(const char *fmt, va_list ap);

/* Chamado pelo CDC_Control_FS quando o terminal abre (1) ou fecha (0) a porta (sinal DTR). */
void monitor_usb_dtr(int aberto);

/* Chamado pelo CDC_Receive_FS (interrupção da USB) com os bytes digitados. */
void monitor_usb_rx(const uint8_t *buf, uint32_t len);

#endif

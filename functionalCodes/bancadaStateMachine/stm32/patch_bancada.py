#!/usr/bin/env python3
"""Variante de bancada da máquina de estados (aplicada só na cópia de build; o repositório não muda).

Na bancada o ESP32 simula o TT&C na USART1 (como no satélite) e o PC lê só a USART2 (ST-Link). Para o PC
conseguir acompanhar tudo pela USART2:
  1. A telemetria ('T' + Telemetry_t) é espelhada na USART2, além da USART1 (o uart.py da branch espera isso).
  2. debugPrintf usa o uartMutex, para uma linha de debug não cair no meio de um quadro de telemetria.
  3. Cada telecomando recebido gera "rx tc=0xNN" no debug (o firmware original só imprime quando algo muda).

Uso: patch_bancada.py <pasta do projeto stateMachineInVSCode>. Falha se algum trecho esperado não existir.
"""
import re
import sys
from pathlib import Path

proj = Path(sys.argv[1])


def patch(rel, pattern, repl, count=1):
    f = proj / rel
    src = f.read_text(encoding="utf-8")
    new, n = re.subn(pattern, repl, src, count=count)
    if n != count:
        sys.exit(f"[patch] trecho não encontrado em {rel}: {pattern!r}")
    f.write_text(new, encoding="utf-8")
    print(f"[patch] {rel}")


# 2. debugPrintf serializado com o quadro de telemetria
patch(
    "Core/Src/main.c",
    r"(#if ENABLE_DEBUG_UART2\n)(\s*)va_list args;\n\s*va_start\(args, format\);\n\s*vprintf\(format, args\);\n\s*va_end\(args\);",
    r"""\1\2/* [bancada] serializa com o quadro de telemetria espelhado na USART2 */
\2bool locked = (osKernelGetState() == osKernelRunning)
\2		&& (osMutexAcquire(uartMutexHandle, osWaitForever) == osOK);
\2va_list args;
\2va_start(args, format);
\2vprintf(format, args);
\2va_end(args);
\2if (locked) {
\2	osMutexRelease(uartMutexHandle);
\2}""",
)

# 3. todo TC recebido aparece no debug
patch(
    "Core/Src/main.c",
    r"(\n\s*bool \*missionActive\)\n\{\n)",
    r"""\1	/* [bancada] registra todo byte recebido, mesmo quando nada muda */
	debugPrintf("[%08lu] rx tc=0x%02X\\r\\n", osKernelGetTickCount(), tc);

""",
)

# 1. telemetria espelhada na USART2
patch(
    "Core/Src/main.c",
    r"(\n(\s*)HAL_UART_Transmit\(&huart1, \(uint8_t\*\) telemetryPacket, sizeof\(Telemetry_t\), 1000\);)",
    r"""\1
\2/* [bancada] espelha o quadro na USART2 (ST-Link) para o PC acompanhar */
\2HAL_UART_Transmit(&huart2, &marker, 1, 100);
\2HAL_UART_Transmit(&huart2, (uint8_t*) telemetryPacket, sizeof(Telemetry_t), 100);""",
)

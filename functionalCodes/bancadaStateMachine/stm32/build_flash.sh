#!/usr/bin/env bash
# Compila o firmware integrado do OBC (branch main) e grava no NUCLEO-L476RG.
#
#   ./build_flash.sh                       # variante de bancada (padrão): compila e grava
#   ./build_flash.sh --monitor             # PC no lugar do TT&C: comandos digitados no monitor serial
#                                          #   (ST-Link, 115200), debug só em texto, telemetria a cada 10 s
#   ./build_flash.sh --monitor-usb         # o mesmo na PCB do OBC (sem ST-Link): pela USB nativa do STM32
#                                          #   (porta COM virtual); ADCS na USART2 como no voo
#   ./build_flash.sh --cubeprog            # grava pelo STM32CubeProgrammer (qualquer ST-Link no SWD), não pelo
#                                          #   drive NOD_L476RG do Nucleo
#   ./build_flash.sh --stflash             # grava pelo st-flash (stlink-tools, código aberto; bom no Linux)
#
# Roda no WSL ou no Linux nativo (detecta sozinho): no WSL grava pelo Windows, no Linux direto pela USB.
#   ./build_flash.sh --original            # código da branch sem nenhuma alteração
#   ./build_flash.sh --no-flash            # só compila
#   ./build_flash.sh --local               # compila o working tree em vez da branch remota (REF)
#
# A variante de bancada (patch_bancada.py) espelha telemetria e debug na USART2 (ST-Link) para o PC
# acompanhar tudo; os TCs chegam pela USART1 (ESP32 simulando o TT&C). Os patches valem só para a cópia de build.
#
# Variáveis: REPO (repositório OBC), REF (branch/commit), PROJ (pasta do projeto na branch).
# O código é extraído com `git archive`, então o working tree do repositório não é alterado.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="${REPO:-$(git -C "$HERE" rev-parse --show-toplevel 2>/dev/null || echo "$HERE/../../OBC")}"
REF="${REF:-origin/main}"
PROJ="${PROJ:-functionalCodes/stateMachineInVSCode}"
OUT="$HERE/build"
SRC="$OUT/src"

VARIANT=bancada
FLASHER=drive
FLASH=1
LOCAL=0
for a in "$@"; do
  case "$a" in
    --original) VARIANT=original ;;
    --monitor) VARIANT=monitor ;;
    --monitor-usb) VARIANT=monitor-usb ;;
    --cubeprog) FLASHER=cubeprog ;;
    --stflash) FLASHER=stflash ;;
    --no-flash) FLASH=0 ;;
    --local) LOCAL=1 ;;
    *) echo "opção desconhecida: $a"; exit 1 ;;
  esac
done

rm -rf "$SRC" && mkdir -p "$SRC"
if [[ "$LOCAL" == 1 ]]; then
  # working tree, com alterações ainda não commitadas
  mkdir -p "$SRC/$PROJ" && (cd "$REPO/$PROJ" && tar -c --exclude=build --exclude=.git . ) | tar -x -C "$SRC/$PROJ"
  COMMIT="$(git -C "$REPO" rev-parse --short HEAD)-local"
  REF="working tree"
else
  git -C "$REPO" archive "$REF" "$PROJ" | tar -x -C "$SRC"
  COMMIT="$(git -C "$REPO" rev-parse --short "$REF")"
fi
P="$SRC/$PROJ"
echo "[build] $REF ($COMMIT), variante $VARIANT -> $OUT"
if [[ "$VARIANT" == bancada ]]; then python3 "$HERE/patch_bancada.py" "$P"; fi
if [[ "$VARIANT" == monitor* ]]; then python3 "$HERE/patch_bancada.py" "$P" --sem-espelho; fi

CPU="-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard"
INC="-I$P/Core/Inc -I$P/Drivers/STM32L4xx_HAL_Driver/Inc -I$P/Drivers/CMSIS/Device/ST/STM32L4xx/Include
     -I$P/Drivers/CMSIS/Include -I$P/Middlewares/Third_Party/FreeRTOS/Source/include
     -I$P/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
     -I$P/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F"
FILES=$(ls "$P"/Core/Src/*.c "$P"/Drivers/STM32L4xx_HAL_Driver/Src/*.c \
           "$P"/Middlewares/Third_Party/FreeRTOS/Source/*.c \
           "$P"/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/*.c \
           "$P"/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/*.c \
           "$P"/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c | grep -v _template)

# shellcheck disable=SC2086
DEFS="-DUSE_HAL_DRIVER -DSTM32L476xx"
if [[ "$VARIANT" == bancada ]]; then DEFS="$DEFS -DBANCADA_ESPELHO=1 -DBANCADA_MISSAO_CONTINUA=1 -DBANCADA_ADCS_UART4=1 -DBANCADA_ADCS_RESET=1"; fi   # P/H/Z na USART2; payload sempre ligado; ADCS em A0/A1
# monitor: TCs digitados na USART2, ADCS em A0/A1, nada binário na USART2, telemetria a cada 10 s, missão por comando
if [[ "$VARIANT" == monitor ]]; then DEFS="$DEFS -DTC_ON_USART2=1 -DBANCADA_ADCS_UART4=1 -DBANCADA_ADCS_RESET=1 -DHK_PERIODO_MS=10000U"; fi
# monitor-usb (PCB): debug e TCs na USB nativa; UARTs como no voo (TT&C 1, ADCS 2, PLD 3)
if [[ "$VARIANT" == monitor-usb ]]; then
  U="$HERE/../usb_monitor"
  DEFS="$DEFS -DMONITOR_USB=1 -DHAL_PCD_MODULE_ENABLED -DHK_PERIODO_MS=10000U"
  INC="$INC -I$U -I$U/app -I$U/lib/Inc -I$U/hal/Inc"
  FILES="$FILES $(ls "$U"/*.c "$U"/app/*.c "$U"/lib/Src/*.c "$U"/hal/Src/*.c)"
fi
# shellcheck disable=SC2086
arm-none-eabi-gcc $CPU $DEFS -std=gnu11 -Og -g3 \
  -ffunction-sections -fdata-sections $INC $FILES "$P/Core/Startup/startup_stm32l476rgtx.s" \
  -T"$P/STM32L476RGTX_FLASH.ld" --specs=nano.specs --specs=nosys.specs \
  -Wl,--gc-sections -Wl,-Map="$OUT/stateMachine.map" \
  -o "$OUT/stateMachine.elf"
arm-none-eabi-objcopy -O binary "$OUT/stateMachine.elf" "$OUT/stateMachine.bin"
arm-none-eabi-size "$OUT/stateMachine.elf"
echo "$COMMIT $VARIANT" > "$OUT/COMMIT"

if [[ "$FLASH" == 0 ]]; then exit 0; fi

# Ambiente: WSL (grava pelo Windows) ou Linux nativo (grava direto pela USB do ST-Link).
if [[ -n "${BANCADA_AMBIENTE:-}" ]]; then AMBIENTE="$BANCADA_AMBIENTE"   # forçar (testes)
elif grep -qi microsoft /proc/version 2>/dev/null || [[ -n "${WSL_DISTRO_NAME:-}" ]]; then AMBIENTE=wsl; else AMBIENTE=linux; fi
BIN="$OUT/stateMachine.bin"
echo "[flash] ambiente $AMBIENTE, método $FLASHER"

if [[ "$FLASHER" == stflash ]]; then
  # stlink-tools (código aberto): sudo apt install stlink-tools. No WSL só com a USB repassada pelo usbipd.
  command -v st-flash >/dev/null || { echo "st-flash não encontrado: sudo apt install stlink-tools"; exit 1; }
  st-flash --reset write "$BIN" 0x08000000
  exit $?
fi

if [[ "$FLASHER" == cubeprog ]]; then
  # STM32CubeProgrammer: ST-Link avulso (V2/V3) ou o do Nucleo (sem os jumpers CN2, cabo no CN4) no SWD da PCB
  CLI="$(command -v STM32_Programmer_CLI || true)"
  if [[ -z "$CLI" && "$AMBIENTE" == wsl ]]; then
    CLI="$(command -v STM32_Programmer_CLI.exe || true)"
    [[ -z "$CLI" ]] && CLI="/mnt/c/Program Files/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI.exe"
  fi
  if [[ -z "$CLI" && "$AMBIENTE" == linux ]]; then   # instalação padrão no Linux fica na home
    for c in "$HOME/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI" /opt/st/stm32cubeprogrammer/bin/STM32_Programmer_CLI; do
      [[ -x "$c" ]] && CLI="$c" && break
    done
  fi
  [[ -n "$CLI" && -x "$CLI" ]] || { echo "STM32_Programmer_CLI não encontrado (instale o STM32CubeProgrammer, ou use --stflash)"; exit 1; }
  [[ "$CLI" == *.exe ]] && BIN="$(wslpath -w "$BIN")"
  "$CLI" -c port=SWD mode=UR -w "$BIN" 0x08000000 -v -rst
  exit $?
fi

# Gravação pelo drive "NOD_L476RG" do ST-Link do Nucleo: copiar o .bin grava a flash e reinicia o STM32.
if [[ "$AMBIENTE" == linux ]]; then
  for d in "/media/$USER/NOD_L476RG" "/run/media/$USER/NOD_L476RG" /media/NOD_L476RG /mnt/NOD_L476RG; do
    if [[ -d "$d" ]]; then cp "$BIN" "$d/" && sync && echo "[flash] gravado em $d"; exit $?; fi
  done
  echo "drive NOD_L476RG não montado (Nucleo conectado? abra no gerenciador de arquivos), ou use --stflash / --cubeprog"
  exit 1
fi
WIN_BIN="$(wslpath -w "$OUT/stateMachine.bin")"
powershell.exe -NoProfile -Command "
  \$v = Get-Volume | Where-Object FileSystemLabel -eq 'NOD_L476RG' | Select-Object -First 1
  if (-not \$v) { Write-Error 'Drive NOD_L476RG não encontrado (Nucleo conectado?)'; exit 1 }
  Copy-Item '$WIN_BIN' (\$v.DriveLetter + ':\\')
  Write-Host ('[flash] gravado em ' + \$v.DriveLetter + ':')
"

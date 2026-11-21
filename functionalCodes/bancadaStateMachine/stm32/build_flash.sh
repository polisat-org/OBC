#!/usr/bin/env bash
# Compila a máquina de estados da branch code_state_machine e grava no NUCLEO-L476RG.
#
#   ./build_flash.sh                       # variante de bancada (padrão): compila e grava
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
REF="${REF:-origin/code_state_machine}"
PROJ="${PROJ:-functionalCodes/stateMachineInVSCode}"
OUT="$HERE/build"
SRC="$OUT/src"

VARIANT=bancada
FLASH=1
LOCAL=0
for a in "$@"; do
  case "$a" in
    --original) VARIANT=original ;;
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

# Gravação: o ST-Link do Nucleo aparece no Windows como um drive "NOD_L476RG";
# copiar o .bin para ele grava a flash e reinicia o STM32.
WIN_BIN="$(wslpath -w "$OUT/stateMachine.bin")"
powershell.exe -NoProfile -Command "
  \$v = Get-Volume | Where-Object FileSystemLabel -eq 'NOD_L476RG' | Select-Object -First 1
  if (-not \$v) { Write-Error 'Drive NOD_L476RG não encontrado (Nucleo conectado?)'; exit 1 }
  Copy-Item '$WIN_BIN' (\$v.DriveLetter + ':\\')
  Write-Host ('[flash] gravado em ' + \$v.DriveLetter + ':')
"

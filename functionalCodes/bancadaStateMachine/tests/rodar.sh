#!/usr/bin/env bash
# Testes no PC: receptor do payload (hermes_protocol.c) + validação/downlink (payload_downlink.c),
# parser do ADCS (adcs_link.c) e leitura de EPS (eps.c + drivers) com I2C simulado.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
CORE="$HERE/../../stateMachineInVSCode/Core"
OUT="$(mktemp -d)"
python3 "$HERE/gera_bloco.py" "$OUT/bloco.bin"
gcc -std=gnu11 -Wall -Wextra -I"$CORE/Inc" -o "$OUT/teste" "$HERE/test_payload_downlink.c" \
    "$CORE/Src/payload_downlink.c" "$CORE/Src/hermes_protocol.c" -lm
"$OUT/teste" "$OUT/bloco.bin" "$OUT/quadroP.bin"
gcc -std=gnu11 -Wall -Wextra -I"$CORE/Inc" -o "$OUT/teste_adcs" "$HERE/test_adcs_link.c" "$CORE/Src/adcs_link.c"
"$OUT/teste_adcs"
gcc -std=gnu11 -Wall -Wextra -I"$HERE/stub_hal" -I"$CORE/Inc" -o "$OUT/teste_eps" "$HERE/test_eps.c" "$CORE/Src/eps.c" \
    "$CORE/Src/ina3221.c" "$CORE/Src/max17048.c" "$CORE/Src/mp2733.c" -lm
"$OUT/teste_eps"

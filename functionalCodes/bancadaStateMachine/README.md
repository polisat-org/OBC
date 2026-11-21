# Bancada do OBC: máquina de estados + ESP32 simulando o TT&C

Testa a máquina de estados do OBC (branch `code_state_machine`, `functionalCodes/stateMachineInVSCode`) com um
ESP32 no papel do TT&C. O ESP32 roda sozinho, mandando telecomandos de um roteiro fixo pela USART1, como no
satélite. O PC **só lê o STM32** (USART2 pelo ST-Link) e mostra tudo num painel web.

```text
 ESP32 (esp32_ttc_mock) ──GPIO17 TX2 → D2/PA10 ──>  NUCLEO-L476RG  ──USB ST-Link (USART2, 115200)──> COM18 ─┐
   roteiro de TCs        <─GPIO16 RX2 ← D8/PA9 ───   (stateMachine,                                         │ relay.ps1
   1 a cada 15 s            USART1 9600 (TT&C)        variante bancada)                                      ▼
                                                                          bridge/server.mjs ──> http://localhost:8780
```

## Ligação

| Nucleo-L476RG | | Liga em | Sinal |
|---|---|---|---|
| **D2** (PA10, USART1_RX) | ← | ESP32 **GPIO17** (TX2) | telecomandos do TT&C |
| **D8** (PA9, USART1_TX) | → | ESP32 **GPIO16** (RX2) | telemetria para o TT&C |
| **GND** | — | ESP32 **GND** | referência comum (obrigatória) |
| **5V** | → | ESP32 **VIN** | alimentação, quando o ESP32 não está no USB |
| **D15** (PB8, I2C1_SCL) | → | SCL do BME280 e do DS3231 | I2C |
| **D14** (PB9, I2C1_SDA) | ↔ | SDA do BME280 e do DS3231 | I2C |
| **3V3 / GND** | → | VCC / GND dos módulos | alimentação dos sensores |

TX de um lado vai sempre no RX do outro. O **D9 é PC7**, que não é UART. D0/D1 não estão ligados ao
microcontrolador no Nucleo-64 (pontes SB62/SB63 abertas de fábrica). O painel tem a mesma tabela na aba
**Ligações e pinos**, com o status de cada fio ao vivo.

## Uso

Fica em `functionalCodes/bancadaStateMachine/` do repositório OBC. O build usa `git archive` da branch
(`REF`, padrão `origin/code_state_machine`), então compila o que está no remoto, não o working tree.

```bash
cd functionalCodes/bancadaStateMachine
./stm32/build_flash.sh        # compila a branch (git archive) + patches da bancada e grava no Nucleo
./stm32/build_flash.sh --original   # grava a branch sem nenhuma alteração
# ESP32, no Windows (arduino-cli da Arduino IDE):
#   & $cli compile --fqbn esp32:esp32:esp32 --upload -p COM13 esp32_ttc_mock
node bridge/server.mjs        # abre a COM18 e serve o painel em http://localhost:8780
```

Só precisa do Node (sem `npm install`). A COM18 não pode estar aberta em outro programa. Opções do bridge:
`--stm COM18 --http 8780 --no-relay`.

## Payload (Raspberry Pi Zero W, HERMES Simples)

| Nucleo | | Raspberry Pi | Sinal |
|---|---|---|---|
| **D6 · PB10** (USART3_TX, CN10 pino 25) | → | pino 10 (GPIO15, RX0) | LIGAR `A5 5A 01 FE` (repete a cada 2 s) / DESLIGAR `A5 5A 00 FF` |
| **PB11** (USART3_RX, CN10 pino 18) | ← | pino 8 (GPIO14, TX0) | 20 × 204 B = 4080 B a cada 5 s |
| **GND** (CN10 pino 20) | — | pino 6 | referência |

Nenhum GPIO é usado. O OBC recebe os blocos por DMA (`hermes_adsb.c`, da equipe do payload), descarta
registros implausíveis (o bloco não tem CRC) e manda ao TT&C um quadro `'P'` a cada 5 s com 28 B por avião
(`payload_downlink.c`). O Pi precisa do SDR no USB dele (porta "USB" do meio, com OTG) e de fonte própria.
Diagrama completo: `web/montagem.html` (ou http://localhost:8780/montagem.html com o bridge rodando).

## Variante de bancada do firmware (`stm32/patch_bancada.py`)

Os patches valem só para a cópia de build; o repositório não muda.

1. A telemetria (`'T'` + 85 B) também sai pela USART2, para o PC ver (o `uart.py` da branch já esperava isso).
   Com `-DBANCADA_ESPELHO=1`, os quadros `'P'` dos aviões também.
2. `debugPrintf` usa o `uartMutex`, para uma linha de debug não cair no meio de um quadro de telemetria.
3. Cada byte recebido gera `rx tc=0xNN` no debug (o original só imprime quando algo muda).

## O painel

- **Visão geral:** resumo de OBC, comandos do TT&C, telemetria e sensores; lista "O que fazer agora" com cada
  problema e a correção; diagrama da máquina de estados com o modo atual; contagem regressiva da próxima
  telemetria e do próximo comando; tasks do FreeRTOS.
- **Comandos do TT&C:** o roteiro do ESP32. Para cada comando recebido, o bridge simula o `handleTelecommand()`
  a partir do estado atual e confere se o debug mostrou exatamente o esperado.
- **Payload:** caminho Pi → OBC → TT&C com o status de cada trecho, tabela e mapa dos aviões do último `'P'`.
- **Telemetria:** relógio, temperatura, pressão, umidade e antena, com tendência; todos os 38 campos do pacote.
- **Ligações e pinos:** diagrama da bancada e tabela de pinos com status ao vivo.
- **Histórico e log:** linha do tempo em português e a USART2 crua.

Cada sessão grava `logs/<data-hora>/debug.log`, `eventos.jsonl` e `telemetria.csv`.

## Se algo mudar

- Roteiro do TT&C: `ROTEIRO` em `esp32_ttc_mock/esp32_ttc_mock.ino` **e** `TTC_SCRIPT` em `bridge/obc.mjs`.
- Layout da telemetria, mensagens de debug e checklist: `bridge/obc.mjs`.
- Lógica de previsão da resposta a um TC: `expectFor()` em `bridge/server.mjs` (espelha o `handleTelecommand()`).
- Deploy: quando a task real for implementada, ponha `ENVIAR_DEPLOY 0` no ESP32 antes de ligar o mecanismo.

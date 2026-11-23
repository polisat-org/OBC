// esp32_ttc_mock — ESP32 simulando o TT&C do CubeSat na bancada do OBC (roda sozinho, sem PC).
//
// Faz, do lado do OBC, o que o cubesat_ttc faz (TTC-Hermes/docs/detalhamento_ttc.md §5):
//   - entrega telecomandos de 1 byte (0x00..0x0F) na USART1 do OBC, como se viessem da estação de solo;
//   - recebe a telemetria 'T' + Telemetry_t que o OBC manda a cada ciclo de housekeeping.
// Os telecomandos seguem um roteiro fixo de "passagem do solo" (ROTEIRO abaixo), que se repete.
// O painel da bancada (bridge/obc.mjs, TTC_SCRIPT) conhece o mesmo roteiro: mantenha os dois iguais.
//
// Ligação (3,3 V, GND comum obrigatório):
//   ESP32 GPIO17 (TX2) -> Nucleo D2 (PA10, USART1_RX)
//   ESP32 GPIO16 (RX2) <- Nucleo D8 (PA9,  USART1_TX)
//   Alimentação sem o USB do PC: ESP32 VIN <- Nucleo 5V, GND <- GND.
//
// LED azul (GPIO2): pulso longo = TC enviado; piscada curta = telemetria recebida.
// GPIO27 = TTC_BUSY, como no cubesat_ttc: alto enquanto o TT&C escreve no OBC e por mais 2 ms.
// Se o USB estiver ligado, um log sai em 115200 (só para depuração; o painel não depende dele).
#include <Arduino.h>

#define OBC_RX_PIN 16
#define OBC_TX_PIN 17
#define OBC_BAUD 9600            // Notion: todas as UARTs a 9600
#define TTC_BUSY_PIN 27
#define LED_PIN 2
#define TELEMETRY_SIZE 85        // corpo após o 'T' (customTypes.h da branch code_state_machine)
#define FRAME_GAP_MS 300

#define START_DELAY_MS 10000     // espera o OBC subir antes do 1º comando
#define STEP_INTERVAL_MS 15000   // um telecomando a cada 15 s

// Deploy: neste firmware do OBC a task é um esqueleto (sem GPIO). Quando o deploy real for implementado,
// troque para 0 antes de usar a bancada com o mecanismo montado.
#define ENVIAR_DEPLOY 1

struct Passo {
  uint8_t tc;
  const char *descricao;
};

// Roteiro da passagem simulada. O telecomando é um vetor de estado (bit0 Mission, bit1 Deploy,
// bit2 Detumbling, bit3 Pointing): bit ausente = desliga a função.
const Passo ROTEIRO[] = {
  {0x00, "solo: tudo desligado"},
  {0x04, "liga detumbling"},
  {0x00, "desliga tudo"},
  {0x08, "liga pointing (azimute)"},
  {0x00, "desliga pointing"},
  {0x01, "liga missao ADS-B"},
  {0x00, "desliga tudo"},
  {0x0C, "detumbling + pointing (detumbling tem prioridade)"},
  {0x09, "pointing + missao"},
  {0x00, "desliga tudo"},
#if ENVIAR_DEPLOY
  {0x02, "deploy da antena (so executa 1x por boot do OBC)"},
#endif
};
const size_t N_PASSOS = sizeof(ROTEIRO) / sizeof(ROTEIRO[0]);

HardwareSerial Obc(2);

size_t passo = 0;
uint32_t ciclo = 0;
uint32_t proximoTc = 0;
uint32_t busyAte = 0, ledAte = 0;

// recepção da telemetria
bool emQuadro = false;
uint8_t corpo[TELEMETRY_SIZE];
size_t nCorpo = 0;
uint32_t ultimoByte = 0;
uint32_t nTelemetria = 0, nIncompletos = 0, nBytes = 0;

void led(uint16_t ms) { digitalWrite(LED_PIN, HIGH); ledAte = millis() + ms; }

void enviaTc(const Passo &p) {
  digitalWrite(TTC_BUSY_PIN, HIGH);
  Obc.write(p.tc);
  Obc.flush();
  busyAte = millis() + 2;
  led(150);
  Serial.printf("[%lu] TC 0x%02X  passo %u/%u ciclo %lu: %s\n", (unsigned long)millis(), p.tc,
                (unsigned)(passo + 1), (unsigned)N_PASSOS, (unsigned long)ciclo, p.descricao);
}

void fimQuadro(bool completo) {
  if (completo) {
    nTelemetria++;
    led(40);
    // hora do RTC do OBC nos 3 primeiros bytes (hora, minuto, segundo)
    Serial.printf("[%lu] telemetria #%lu recebida (%02u:%02u:%02u no RTC do OBC)\n", (unsigned long)millis(),
                  (unsigned long)nTelemetria, corpo[0], corpo[1], corpo[2]);
  } else {
    nIncompletos++;
    Serial.printf("[%lu] telemetria incompleta: %u de %u B\n", (unsigned long)millis(), (unsigned)nCorpo,
                  (unsigned)TELEMETRY_SIZE);
  }
  emQuadro = false;
  nCorpo = 0;
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(TTC_BUSY_PIN, OUTPUT);
  digitalWrite(TTC_BUSY_PIN, LOW);
  Serial.begin(115200);
  Obc.begin(OBC_BAUD, SERIAL_8N1, OBC_RX_PIN, OBC_TX_PIN);
  Obc.setRxBufferSize(512);
  proximoTc = millis() + START_DELAY_MS;
  Serial.printf("\nesp32_ttc_mock: %u passos, 1 TC a cada %u s, 1o em %u s\n", (unsigned)N_PASSOS,
                STEP_INTERVAL_MS / 1000, START_DELAY_MS / 1000);
}

void loop() {
  uint32_t agora = millis();

  while (Obc.available()) {
    uint8_t b = Obc.read();
    nBytes++;
    ultimoByte = agora;
    if (emQuadro) {
      corpo[nCorpo++] = b;
      if (nCorpo == TELEMETRY_SIZE) fimQuadro(true);
    } else if (b == 'T') {
      emQuadro = true;
      nCorpo = 0;
    }
  }
  if (emQuadro && agora - ultimoByte > FRAME_GAP_MS) fimQuadro(false);

  if ((int32_t)(agora - proximoTc) >= 0) {
    enviaTc(ROTEIRO[passo]);
    passo++;
    if (passo == N_PASSOS) { passo = 0; ciclo++; }
    proximoTc = agora + STEP_INTERVAL_MS;
  }

  if (busyAte && agora >= busyAte) { digitalWrite(TTC_BUSY_PIN, LOW); busyAte = 0; }
  if (ledAte && agora >= ledAte) { digitalWrite(LED_PIN, LOW); ledAte = 0; }
}

int f = 60; //sinal da frequencia
float fs = 500.0; //mostra da frequencia gerada pelo pino
int sig[500]; //matriz de sinais
float t; //tempo
float doispi = 6.28;

void setup() {
  pinMode(5, OUTPUT);
  //gerar sinal seno
  for (int i = 0; i<500; i++){
      t = (float)i/fs;
      sig[i] = (int)(127.0*(sin(doispi * f * t) + 1.0));
  }
}
void loop() {
  for (int i = 0; i<500; i++){
    analogWrite(5, sig[i]);
    delay(2);
  }

}

#define RX2 16
#define TX2 17

#include <stdio.h>

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RX2, TX2);
  // pinMode(v, OUTPUT);
  // pinMode(a, OUTPUT);
  // put your setup code here, to run once:

}

int counter = 1;
void loop() {
  Serial.printf("Enviado counter %d \r\n", counter);
  Serial2.write(counter);
  counter++;
  delay(2000);
  // put your main code here, to run repeatedly:

}

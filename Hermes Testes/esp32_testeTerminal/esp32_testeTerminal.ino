#define v 18
#define a 19

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  // pinMode(v, OUTPUT);
  // pinMode(a, OUTPUT);
  // put your setup code here, to run once:

}

int counter = 1;
void loop() {
  Serial.print(0x02);
  Serial2.write(0x02);

  // if (counter == 1) {
  //   digitalWrite(v, 1); 
  //   digitalWrite(a, 0);
  //   counter = 2;
  // } else if (counter == 2) {
  //   digitalWrite(v, 0); 
  //   digitalWrite(a, 1);
  //   counter = 1;
  // } else counter = 0;
  delay(2000);
  // put your main code here, to run repeatedly:

}

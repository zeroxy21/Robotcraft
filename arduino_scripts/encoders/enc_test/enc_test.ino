#include <Encoder.h>

#define ENC_M1R_A 18
#define ENC_M1R_B 19

Encoder enc(ENC_M1R_A, ENC_M1R_B);

void setup() {
  Serial.begin(9600);
}

void loop() {
  long pos = enc.read();
  Serial.println(pos);
  delay(100);
}

#include "Piston.h"
Piston p1(12,11);

void setup() {
  Serial.begin(9600);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  p1.extendPiston();
}



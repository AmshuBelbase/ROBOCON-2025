#include "Piston.h"
#include <Arduino.h>

Piston::Piston(int extend, int retract) {
  this->extend = extend;
  this->retract = retract;

  pinMode(extend, OUTPUT);
  pinMode(retract, OUTPUT);

  retractPiston();
  state = false;
}

void Piston::pistonControl(bool state) {  // 1 - extended, 0 - retracted
  digitalWrite(this->extend, state);
  digitalWrite(this->retract, !state);
  delay(500);
  digitalWrite(this->extend, LOW);
  digitalWrite(this->retract, LOW);
  this->state = state;
}

void Piston::extendPiston() {
  digitalWrite(this->extend, HIGH);
  digitalWrite(this->retract, LOW);
  delay(500);
  digitalWrite(this->extend, LOW);
  digitalWrite(this->retract, LOW);
  state = true;
}

void Piston::retractPiston() {
  digitalWrite(this->extend, LOW);
  digitalWrite(this->retract, HIGH);
  delay(500);
  digitalWrite(this->extend, LOW);
  digitalWrite(this->retract, LOW);
  state = false;
}

void Piston::toggle() {
  if (state) {
    retractPiston();
  } else {
    extendPiston();
  }
}

void Piston::getStatus() {
  Serial.println(state ? "Extended" : "Retracted");
}
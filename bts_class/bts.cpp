#include "bts.h"

bts::bts(int leftpwm, int rightpwm) {
  lpwm=leftpwm;
  rpwm=rightpwm;
  pinMode(rpwm, OUTPUT);
  pinMode(lpwm, OUTPUT);
}

void bts::drive(int pwmval) {
  if (pwmval > 0) {
    analogWrite(rpwm, pwmval);
    analogWrite(lpwm, 0);
  } else if (pwmval < 0) {
    analogWrite(rpwm, 0);
    analogWrite(lpwm, abs(pwmval));
  } else {
    analogWrite(rpwm, 0);
    analogWrite(lpwm, 0);
  }
}
#include "Arduino.h"
#include "cytron.h"

cytron::cytron(int pwmPin, int dirPin) {  //for arduino
  pwm = pwmPin;
  dir = dirPin;
  pinMode(pwm, OUTPUT);
  pinMode(dir, OUTPUT);
}

cytron::cytron(int pwmPin, int dirPin, int analogfrequency){  //for teensy
  pwm = pwmPin;
  dir = dirPin;
  pinMode(pwm, OUTPUT);
  pinMode(dir, OUTPUT);
  analogWriteResolution(14);
  analogWriteFrequency(pwm, 9000);
}

void cytron::run(int pwmval) {   //for teensy
  pwmval = constrain(pwmval, -16383, 16383);
  digitalWrite(dir, (pwmval <= 0 ? LOW : HIGH));
  analogWrite(pwm, abs(pwmval));
}

void cytron::drive_8bit(int pwmval){   //for arduino
  pwmval = constrain(pwmval, -255, 255);
  digitalWrite(dir, (pwmval <= 0 ? LOW : HIGH));
  analogWrite(pwm, abs(pwmval));
}


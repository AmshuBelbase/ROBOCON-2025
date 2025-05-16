#ifndef CYTRON_H
#define CYTRON_H

#include <Arduino.h>

class cytron {
  public:
    cytron(int pwmPin, int dirPin);
    cytron(int pwmPin, int dirpin , int analogfrequency);
    void run(int pwmval);
    void drive_8bit(int pwmval);
  
  private:
    int pwm;
    int dir;
};

#endif

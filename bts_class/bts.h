#ifndef BTS_H
#define BTS_H

#include <Arduino.h>

class bts {
public:
  bts(int leftpwm, int rightpwm); 
  void drive(int pwmval); 

private:
  int rpwm;
  int lpwm;
  int pwmval;
};
#endif
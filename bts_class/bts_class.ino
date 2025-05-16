#include "bts.h"

bts motor1(7,9); //leftpwm,rightpwm


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  motor1.drive(255);

}

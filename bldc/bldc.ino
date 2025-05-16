/*
  Name:    setCurrent.ino
  Created: 19-08-2018
  Author:  SolidGeek
  Description: This is a very simple example of how to set the current for the motor
*/

#include <VescUart.h>

/** Initiate VescUart class */
VescUart UART;

float current = 0; /** The current in amps */

void setup() {
  Serial.begin(2000000);
  /** Setup UART port (Serial1 on Atmega32u4) */
  Serial1.begin(115200);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  
  while (!Serial1) {;}

  /** Define which ports to use as UART */
  UART.setSerialPort(&Serial1);
}
  float rpm=0;


void loop() {
  
  /** Call the function setCurrent() to set the motor current */

    if (Serial.available() > 0) {
                 String input = Serial.readString();
                // Serial.println(input.length());
      
                   rpm=input.toFloat()*7;
                 
               }
//                Serial.print()
// Serial.print(current);
  //UART.setCurrent(current);

    UART.setRPM(rpm);
  if ( UART.getVescValues() ) {
    Serial.println("Values: ");
    Serial.println(UART.data.rpm/7);
    Serial.println(UART.data.inpVoltage);
    Serial.println(UART.data.ampHours);
    Serial.println(UART.data.pidPos);

    Serial.println(UART.data.avgMotorCurrent);
    Serial.println(UART.data.avgInputCurrent);

    Serial.println(UART.data.tachometer);

  }

  // UART.setBrakeCurrent(current);
  
}
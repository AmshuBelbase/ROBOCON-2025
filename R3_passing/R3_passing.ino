/*
  Name:    setCurrent.ino
  Created: 19-08-2018
  Author:  SolidGeek
  Description: This is a very simple example of how to set the current for the motor
*/

#include <VescUart.h>

/** Initiate VescUart class */
VescUart UART;
#include "USBHost_t36.h"

int motor_dir_pin=5;
int motor_pwm_pin=4;
int limitswitch_up=19;
int limitswitch_down=17;
int counter=0;
bool switchup=1;
bool switchdown=0;
 
USBHost myusb;
USBHub hub1(myusb);
USBHIDParser hid1(myusb);
JoystickController joystick1(myusb);
// BluetoothController bluet (myusb, true, "0000");   // Version does pairing to device
BluetoothController bluet(myusb);  // version assumes it already was paireduint32_t buttons_prev = 0;
uint32_t buttons;

int psAxis[64];
int psAxis_prev[64];
bool first_joystick_message = true;

float current = 0; /** The current in amps */

void setup() {
  Serial.begin(2000000);
  /** Setup UART port (Serial1 on Atmega32u4) */
  Serial1.begin(115200);
  
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

  ball_feed();

  // UART.setBrakeCurrent(current);
  
}
void ball_feed(){
   myusb.Task();
   if (joystick1.available()) {
    for (uint8_t i = 0; i < 64; i++) {
      psAxis_prev[i] = psAxis[i];
      psAxis[i] = joystick1.getAxis(i);
    }
     buttons = joystick1.getButtons();
    //  Serial.println(buttons);
    }
  switchup = digitalRead(limitswitch_up);
  switchdown = digitalRead(limitswitch_down);
  // Serial.print("up:");
  // Serial.print(switchup);
  // Serial.print("down:");
  // Serial.println(switchdown);
 
  // Serial.println(counter); 
  if(counter==0){
    if(buttons==4){
      digitalWrite(motor_dir_pin,HIGH);
      analogWrite(motor_pwm_pin,32*255);
      // Serial.println("SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSStart");

    }
    if(switchup==0){
      digitalWrite(motor_dir_pin,LOW);
      analogWrite(motor_pwm_pin,0);
      // Serial.println("SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSStop");
      counter=1;

    }
  }
  else if(counter==1){
    if(buttons==4){
      digitalWrite(motor_dir_pin,LOW);
      analogWrite(motor_pwm_pin,32*255);
    }
    if(switchdown==0){
      digitalWrite(motor_dir_pin,HIGH);
      analogWrite(motor_pwm_pin,0);
      // Serial.println("SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS");
      counter=0;
    }
  }
 
}
#include "USBHost_t36.h"

/*
  Name:    setCurrent.ino
  Created: 19-08-2018
  Author:  SolidGeek
  Description: This is a very simple example of how to set the current for the motor
*/

#include <VescUart.h>
#include <Encoder.h>

//PS4 connection 
USBHost myusb;   // initializes and manages the USB host port, enabling Teensy to detect and commuincate with USB bluetooth dongle
USBHIDParser hid1(myusb);  //works behind the scenes to parse HID data that comes from the PS4 controller, such as joystick movements and button presses.
JoystickController joystick1(myusb);
// BluetoothController bluet(myusb, true, "0000"); 
BluetoothController bluet(myusb);   // Version does pairing to device
uint32_t buttons;

/** Initiate VescUart class */
VescUart UART;

IntervalTimer feed_pid_timer;
float current = 0; /** The current in amps */

Encoder encFeed(7,6);
// int feed = 0;

int feeder_pwm=24;
int feeder_dir=12;

int feeder_cpr=538;
float ap_count_feeder=0;
float sp_angle_feeder=0;
float sp_count_feeder=0;

float kP=150,kI=5,kD=1;

float err_feed=0,der_feed=0,integ_feed=0,lastError_feed=0;
float speed_feed=0;


void setup() {
  Serial.begin(2000000);
  /** Setup UART port (Serial1 on Atmega32u4) */
  Serial1.begin(115200);
  
  // while (!Serial8) {;}
  analogWrite(13,255);
  /** Define which ports to use as UART */
  UART.setSerialPort(&Serial1);

  // feed_pid_timer.begin(feed_pos_pid,10000);

  pinMode(feeder_pwm,OUTPUT);
  pinMode(feeder_dir,OUTPUT);

  analogWriteResolution(14);
  analogWriteFrequency(0, 9000);

  Serial.println("\n\nUSB Host Testing - Joystick Bluetooth");
  if (CrashReport) Serial.print(CrashReport);
  myusb.begin();
  myusb.Task();
}
  int nrpm=0;
  int orpm = 0;

void setPosition(int pwm,int dir ,int speed)
{
  digitalWrite(dir,speed<0?LOW:HIGH);
  analogWrite(pwm,abs(speed));
}

void feed_pos_pid()
{

ap_count_feeder=encFeed.read();
sp_count_feeder=sp_angle_feeder*feeder_cpr/360.0;

err_feed=sp_count_feeder-ap_count_feeder;
der_feed=(err_feed-lastError_feed)/0.01;
integ_feed=integ_feed+(err_feed-lastError_feed)*0.01;
lastError_feed=err_feed;

speed_feed=kP*err_feed+kI*integ_feed+kD*der_feed;
constrain(speed_feed,-14361,14361);
// Serial.printf("speed: %f   ",speed_m);
// Serial.printf("Kp: %f.   Ki: %f.    Kd: %f   ",kp,ki,kd);
// Serial.print("Sp_feed:");
// Serial.print(sp_angle_feed);
// Serial.print("     Ap_feed:");
// Serial.println(int((ap_angle_feed*360.0/shooter_rotation_cpr)));
// //       // Serial.println(enc1.read());
setPosition(feeder_pwm,feeder_dir,int(speed_feed));

}


void loop() {
  myusb.Task();
  if (joystick1.available()) {
    buttons = joystick1.getButtons();
  }
  /** Call the function setCurrent() to set the motor current */

    if (Serial.available() > 0) {
      String input = Serial.readString();//15001
    // Serial.println(input.length());

      nrpm=input.toFloat()*7;
      // feed=input.substring(5).toInt();
      
    }
    if(nrpm != 0){
      // sp_angle_feeder = -3300;
      while(orpm < nrpm){
        orpm=orpm+((nrpm-orpm)/700)+((nrpm-orpm)%700);
        UART.setRPM(orpm);
        delay(50);
        Serial.println("1");
        Serial.println((orpm/7));
      }
      while(orpm > nrpm){
        orpm=orpm-((orpm-nrpm)/700)-((orpm-nrpm)%700);
        UART.setRPM(orpm);
        delay(50);
        Serial.println("2");
        Serial.println((orpm/7));
      } 
    }else{
      orpm = 0;
      // sp_angle_feeder = 0;

    }
    UART.setRPM(orpm);
    if ( UART.getVescValues() ) {
      if(UART.data.avgInputCurrent > 15){
        Serial.println("Values: ");
        // Serial.println(UART.data.rpm/7);
        // Serial.println((UART.data.inpVoltage)/10);
        // Serial.println(UART.data.ampHours);
        // Serial.println(UART.data.pidPos);
        // Serial.println(UART.data.avgMotorCurrent);
        Serial.println(UART.data.avgInputCurrent);
        // Serial.println(UART.data.tachometer);
    }
  }
  if (buttons == 8){
    digitalWrite(feeder_dir, HIGH);
    analogWrite(feeder_pwm, 255*48);

  }else if (buttons == 2){
    digitalWrite(feeder_dir, LOW);
    analogWrite(feeder_pwm, 255*48);
  }else{
    digitalWrite(feeder_dir, HIGH);
    analogWrite(feeder_pwm, 0);
  }

//                Serial.print()
// Serial.print(current);
  //UART.setCurrent(current);
  // if (feed == 1){
  //   UART.setRPM(orpm);
  //   sp_angle_feeder=3550;
  // }else if (feed == 0){
  //   UART.setRPM(0);
  //   sp_angle_feeder = 0;

  // }


    

  // if ( UART.getVescValues() ) {
  //   Serial.println("Values: ");
  //   Serial.println(UART.data.rpm/7);
  //   Serial.println(UART.data.inpVoltage);
  //   Serial.println(UART.data.ampHours);
  //   Serial.println(UART.data.pidPos);

  //   Serial.println(UART.data.avgMotorCurrent);
  //   Serial.println(UART.data.avgInputCurrent);

  //   Serial.println(UART.data.tachometer);

  // }

  // UART.setBrakeCurrent(current);
  
}
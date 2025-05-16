#include "USBHost_t36.h"
#include <Encoder.h>
IntervalTimer rpm_timer;
IntervalTimer ps4_timer;
const int ledPin = LED_BUILTIN;  // the pin with a LED
Encoder myEnc(22,23); // m1
int m4_pwm = 4;
int m4_dir = 3;

// Encoder myEnc(20,21);  // m2
// int m4_pwm = 6;
// int m4_dir = 5;

// Encoder myEnc(18,17);   // m3
// int m4_pwm = 10;
// int m4_dir = 11;

// Encoder myEnc(36,37);  // turn table 
// int m4_pwm = 18;
// int m4_dir = 17;

volatile float rpm=0;
int val = 0;

int res=pow(2,14)-1;
int duty_cycle=25;//in percentage
int max_pwm =(int)(duty_cycle/100.0*res); //6v--250rpm

//int res=pow(2,14)-1;
//int duty_cycle=25;//in percentage
//int max_pwm =(int)(duty_cycle/100.0*res); //6v--250rpm
void drive(int pwm_val, int pwmPin, int dirPin)
{
  
  digitalWrite(dirPin, (pwm_val <= 0 ? LOW : HIGH));
 // Serial.printf("Dir: %d\n", (pwm_val <= 0 ? LOW : HIGH));
  //Serial.printf("PWM: %d \n", abs(pwm_val),);
  analogWrite(pwmPin, abs(pwm_val));
}

// USBHost myusb;
// JoystickController joystick1(myusb);
// // BluetoothController bluet(myusb, true, "0000");
// BluetoothController bluet(myusb);   // version assumes it already was paireduint32_t buttons_prev = 0;

uint32_t buttons_prev = 0;
uint32_t buttons;

int psAxis[64];
int psAxis_prev[64];
bool first_joystick_message = true;



void setup() {

  Serial.begin(9600);
  // while (!Serial) ; // wait for Arduino Serial Monitor

  Serial.println("\n\nUSB Host Testing - Joystick Bluetooth");
  if (CrashReport) Serial.print(CrashReport);
  // myusb.begin();

 // analogWriteFrequency(m2_pwm, 9000);
  
  pinMode(13, OUTPUT);
  digitalWrite(13,HIGH);
  pinMode(m4_dir,OUTPUT);
  
  Serial.begin(9600);
  ps4_timer.priority(0);
  rpm_timer.priority(1);
  ps4_timer.begin(ps4_input,75000);
  rpm_timer.begin(calc_rpm, 75000);// calcRpm() to run every 0.1 seconds....us
  analogWriteResolution(14);

  analogWriteFrequency(m4_pwm, 9000);
  
//  analogWrite(m4_pwm,6000);
//  Serial.println("move");
//  analogWrite(m4_dir,res);
  
}
 long oldPosition  = 0;
int ledState = LOW;
volatile unsigned long count = -999; // use volatile for shared variables
volatile long newPosition;


void calc_rpm() {
  newPosition=myEnc.read();
  count=abs(newPosition-oldPosition);
  rpm=count/1300.0*600*4/3;
  rpm*=newPosition<oldPosition?-1:1;
  Serial.printf("RPM: %f   ",rpm);
  count=0;
  oldPosition=newPosition;
//Serial.println(myEnc.read());
//
//Serial.println(millis());
}


void ps4_input(){
  // String input = Serial.readStringUntil('\n');

  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    val=input.toInt();
                 
  }
// myusb.Task();
//   if (joystick1.available()) {
//     for (uint8_t i = 0; i < 64; i++) {
//       psAxis_prev[i] = psAxis[i];
//       psAxis[i] = joystick1.getAxis(i);
//     }

//     buttons = joystick1.getButtons();

//     int psAxisX=0;
// int psAxisY=0;
// int w= 0;
// if(psAxis[0]<125)
//          psAxisX=map(psAxis[0],125,0,0,255);
      
//       else if(psAxis[0]>135)
//         psAxisX=map(psAxis[0],135,255,0,-255);
//       else
//          psAxisX=0;
      
//       if(psAxis[1]>135)
//         psAxisY=map(psAxis[1],135,255,0,-255);
      
//       else if(psAxis[1]<125)
//          psAxisY=map(psAxis[1],125,0,0,255);
//       else
//          psAxisY=0;
//       if(psAxis[2]>135)
//         w=map(psAxis[2],135,255,0,-255);
      
//       else if(psAxis[2]<125)
//          w=map(psAxis[2],125,0,0,255);
//       else
//          w=0;
//     int y = psAxisY;
//     int x= psAxisX;
          Serial.println(val);
//             //int y = map(left_y, -128, 128,max_pwm, -max_pwm);
          Serial.printf(" PWM:  %d\n", val*255);
          drive(255*val, m4_pwm, m4_dir);

  }

          
float rpmCopy=0;
// The main program will print the blink count
// to the Arduino Serial Monitor
void loop() {

    
          

//ps4_input();
  

 
}
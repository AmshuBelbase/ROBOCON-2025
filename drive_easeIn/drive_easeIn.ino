#include <Encoder.h>
#include <IntervalTimer.h> 
#include "USBHost_t36.h"

//PS4 connection 
USBHost myusb;   // initializes and manages the USB host port, enabling Teensy to detect and commuincate with USB bluetooth dongle
USBHIDParser hid1(myusb);  //works behind the scenes to parse HID data that comes from the PS4 controller, such as joystick movements and button presses.
JoystickController joystick(myusb);
BluetoothController bluet(myusb, true, "0000");     // Version does connecting to device
// BluetoothController bluet(myusb);   // Version does pairing to device

//coordinates of joystick (x,y -> right joystick; leftX -> left joystick)
int x = 0, y = 0, leftX = 0;  
float rad, mag, new_mag;

float max_rpm = 100;

int max_xy = 100;
float max_mag = sqrt(max_xy*max_xy + max_xy*max_xy);

IntervalTimer timer; // Timer object for periodic execution

#include <math.h>
#define M_PI 3.14159265358979323846

double easeInOutSine(double x) {
    return -(cos(M_PI * x) - 1) / 2;
}

float easeInSine(float x) {
    return 1 - cos((x * M_PI) / 2);
}

float easeInQuad(float x) {
    return x*x;
}


float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {

    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

}

void calculatePID() {

  myusb.Task();   // Handle USB host tasks

  if (joystick.available()) {

    // Left Stick values (axes 0 and 1)
    int leftStickX = joystick.getAxis(0);
    leftX = map(leftStickX, 0, 255, -100, 100);

    // Right Stick values (axes 2 and 5)
    int rightStickX = joystick.getAxis(2);
    x = map(rightStickX, 0, 255, -100, 100);

    int rightStickY = joystick.getAxis(5);
    y = map(rightStickY, 0, 255, 100, -100);
    // x = 20;
    // y = 50;


    // rad = atan2(y, x);
    mag = sqrt(x * x + y * y);

    if (x == 0){
      rad = 1.5708;
    }else{
      rad = atan(fabs(y) / fabs(x));
    }

    // Serial.printf(" rad:%f\n",rad);
    // Serial.printf(" mag:%f",mag);
    

    new_mag = mapFloat(easeInSine(mapFloat(mag, 0, max_mag, 0, 1)), 0, 1, 0, max_mag);
    Serial.printf(" new_mag:%f",new_mag);

    float newx = cos(rad) * new_mag;
    float newy = sin(rad) * new_mag;

    // Serial.printf(" newx:%f\n",newx);
    // Serial.printf(" newy:%f",newy);

    if (x < 0) newx *= -1;
    if (y < 0) newy *= -1;

    Serial.printf(" x:%d\n",newx);
    Serial.printf(" y:%d",newy);
    // Serial.printf(" left x:%d",leftX);
  }



  
}



void setup() {
  Serial.begin(9600);

  analogWriteResolution(14);
  analogWriteFrequency(0, 9000);

  // myusb.begin();
  // delay(2000);

  //UART.setSerialPort(&Serial1);
  Serial.println("\n\nUSB Host Testing - Joystick Bluetooth");
  if (CrashReport) Serial.print(CrashReport);
  myusb.begin();
  myusb.Task();

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  timer.begin(calculatePID, 75000);
}

void loop() {
  // digitalWrite(0, HIGH);
  // analogWrite(1, 12000);

  // digitalWrite(4, HIGH);
  // analogWrite(5, 12000);

  // digitalWrite(6, HIGH);
  // analogWrite(7, 12000);

}
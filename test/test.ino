#include <Encoder.h>
#include "USBHost_t36.h"

Encoder myEnc1(14, 15);
Encoder myEnc2(39, 38);
Encoder myEnc3(40, 41);
Encoder myEnc4(0, 1);
Encoder myEnc5(2, 3);

// Motor driver pins
int IN1 = 20; // Motor 1
int IN2 = 17; // Motor 2
int IN3 = 21; // Motor 3

// PWM pins for speed control
int ENA = 22; // Motor 1 PWM pin
int ENB = 19; // Motor 2 PWM pin
int ENC = 23; // Motor 3 PWM pin

volatile long encoderCounts = 0;  // Shared variable to track encoder counts
volatile long lastCount1, lastCount2, lastCount3, lastCount4, lastCount5= 0;      
volatile double rpm1, rpm2, rpm3, rpm4, rpm5 = 0;          // Stores the calculated RPM
long positionChange1, positionChange2, positionChange3, positionChange4, positionChange5;

//PS4 connection 
USBHost myusb;   // initializes and manages the USB host port, enabling Teensy to detect and commuincate with USB bluetooth dongle
USBHIDParser hid1(myusb);  //works behind the scenes to parse HID data that comes from the PS4 controller, such as joystick movements and button presses.
JoystickController joystick(myusb);
BluetoothController bluet(myusb, true, "0000");   // Version does pairing to device

//coordinates of joystick (x,y -> right joystick; leftX -> left joystick)
int x, y, leftX;  

void setup() {
  // Initialize motor control pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(ENC, OUTPUT);

  analogWriteResolution(14);
  analogWriteFrequency(0, 9000);

  //teensy led
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  // Initialize serial communication for debugging
  Serial.begin(9600);
  
  myusb.begin();
  delay(2000);

  // Setting each motor at low
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  analogWrite(ENC, 0);
}

void loop() {
  
  myusb.Task(); 
  if (joystick.available()) {

    // Left Stick values (axes 0 and 1)
    int leftStickX = joystick.getAxis(0);
    leftX = map(leftStickX, 0, 255, -127, 127);
    

    // Right Stick values (axes 2 and 5)
    int rightStickX = joystick.getAxis(2);
    x = map(rightStickX, 0, 255, -127, 127);
    int rightStickY = joystick.getAxis(5);
    y = map(rightStickY, 0, 255, 127, -127);

    //to ignore small joystick values
    if (abs(x) < 5) x = 0;
    if (abs(y) < 5) y = 0;
    if (abs(leftX) < 5) leftX = 0;
  }
  else{
    Serial.print("Joystick Not Found");
    delay(500);
  }

  long currentCounts1 = myEnc1.read();
  positionChange1 = currentCounts1 - lastCount1;
  lastCount1 = currentCounts1;

  long currentCounts2 = myEnc2.read();
  positionChange2 = currentCounts2 - lastCount2;
  lastCount2 = currentCounts2;

  long currentCounts3 = myEnc3.read();
  positionChange3 = currentCounts3 - lastCount3;
  lastCount3 = currentCounts3;

  long currentCounts4 = myEnc4.read();
  positionChange4 = currentCounts4 - lastCount4;
  lastCount4 = currentCounts4;

  long currentCounts5 = myEnc5.read();
  positionChange5 = currentCounts5 - lastCount5;
  lastCount5 = currentCounts5;



  // Calculate RPM
  rpm1 = (positionChange1 / 1300.0) * (60 * (1000.0 / 75));
  rpm2 = (positionChange2 / 1300.0) * (60 * (1000.0 / 75));
  rpm3 = (positionChange3 / 1300.0) * (60 * (1000.0 / 75));
  rpm4 = (positionChange4 / 17500.0) * (60 * (1000.0 / 75));
  rpm5 = (positionChange5 / 17500.0) * (60 * (1000.0 / 75));


  Serial.print("rpm1: ");
  Serial.println(rpm1);
  Serial.print(" rpm2: ");
  Serial.println(rpm2);
  Serial.print(" rpm3: ");
  Serial.println(rpm3);
  Serial.print(" rpm4: ");
  Serial.println(rpm4);
  Serial.print(" rpm5: ");
  Serial.println(rpm5);

  // Calculate wheel speeds based on inverse kinematics
    int V1 = ((x) * (-0.67) + (y) * 0 + (leftX) * (0.33));        
    int V2 = ((x) * (0.33) + (y) * (-0.57) + (leftX) * (0.33)); 
    int V3 = ((x) * (0.33) + (y) * (0.57) + (leftX) * (0.33)); 



  // Serial.print("V1: ");
  // Serial.println(V1);
  // Serial.print("V2: ");
  // Serial.println(V2);
  // Serial.print("V3: ");
  // Serial.println(V3);

  // Set motor speeds based on calculated velocities
  runMotor(IN1, ENA, V1);
  runMotor(IN2, ENB, V2);
  runMotor(IN3, ENC, V3);

  delay(200);  // Small delay for stability


}

void runMotor(int IN, int EN, float speed) {
  int pwmValue = map(abs(speed), 0, 127, 0, 16383);
  pwmValue = constrain(pwmValue, 0, 8000); // Ensure pwmValue is between 0 and 255
  if (speed > 0) {      //to check direction: if +ve - HIGH, else LOW
    digitalWrite(IN, HIGH);
  } else if (speed < 0) {
    digitalWrite(IN, LOW);
  } else {
    pwmValue = 0;
  }
  analogWrite(EN, pwmValue);
}


#include <Encoder.h>
#include <IntervalTimer.h> 
#include "USBHost_t36.h"

Encoder myEnc[3] = {Encoder(8,9), Encoder(12,11), Encoder(28,27)};

//PS4 connection 
USBHost myusb;   // initializes and manages the USB host port, enabling Teensy to detect and commuincate with USB bluetooth dongle
USBHIDParser hid1(myusb);  //works behind the scenes to parse HID data that comes from the PS4 controller, such as joystick movements and button presses.
JoystickController joystick(myusb);
BluetoothController bluet(myusb);   // Version does pairing to device

//coordinates of joystick (x,y -> right joystick; leftX -> left joystick)
int x, y, leftX;  
int PWM[3] = {5,1,3};
int DIR[3] = {4,0,2};

long currentCounts[3] = {0,0,0};
volatile long lastCount[3] = {0,0,0};      
volatile double rpm[3] = {0,0,0};          // Stores the calculated RPM
long positionChange[3] = {0,0,0};

//pid constants
float kp[3] = {0.0, 0.0, 0.0};
float ki[3] = {0.0, 0.0, 0.0};
float kd[3] = {0.0, 0.0, 0.0};   


float pid[3] = {0.0, 0.0, 0.0};
float err[3] = {0.0, 0.0, 0.0};
float prev_err[3] = {0.0, 0.0, 0.0};
float integ[3] = {0.0, 0.0, 0.0};
float der[3] = {0.0, 0.0, 0.0};

float max_rpm = 200;

IntervalTimer timer; // Timer object for periodic execution

void input() {
  if (Serial.available() > 0) {
    String input = Serial.readString();
    i = input.substring(0,1).toInt();
    kp[i] = input.substring(2,9).toFloat();
    ki[i] = input.subtring(10,17).toFloat();
    kd[i] = input.substring(18,24).toFloat();

    // int comma1 = input.indexOf(',');
    // int comma2 = input.indexOf(',', comma1 + 1);
    
    // if (comma1 > 0 && comma2 > comma1) {
    //   kp = input.substring(0, comma1).toFloat();
    //   ki = input.substring(comma1 + 1, comma2).toFloat();
    //   kd = input.substring(comma2 + 1).toFloat();
    // }
  }
}


// void input() {
//   if (Serial.available() > 0) {
  
//   String input = Serial.readString();

//   kp = input.substring(0,3).toFloat();    //20
//   ki = input.substring(3,6).toFloat();    //100
//   kd = input.substring(6).toFloat();      //0.5
//   }

// }

void calculatePID() {
  // unsigned long startTime = micros();

  input();

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

    //to ignore small joystick values
    if (abs(x) < 5) x = 0;
    if (abs(y) < 5) y = 0;
    if (abs(leftX) < 5) leftX = 0;
  }
  // else{
  //  // Serial.print("Joystick Not Found");
  //   delay(500);
  // }

  // Serial.print("x: ");
  // Serial.println(x);
  // Serial.print("y: ");
  // Serial.println(y);
  // Serial.print("leftx: ");
  // Serial.println(leftX);



  float V1 = ((x) * (-0.67) + (y) * 0 + (leftX) * (0.33));        
  float V2 = ((x) * (0.33) + (y) * (-0.57) + (leftX) * (0.33)); 
  float V3 = ((x) * (0.33) + (y) * (0.57) + (leftX) * (0.33)); 

  // Serial.print("V1: ");
  // Serial.println(V1);
  // Serial.print("V2: ");
  // Serial.println(V2);
  // Serial.print("V3: ");
  // Serial.println(V3);

  V1 = map(V1, -100, 100, -max_rpm, max_rpm);
  V2 = map(V2, -100, 100, -max_rpm, max_rpm);
  V3 = map(V3, -100, 100, -max_rpm, max_rpm);

  float sp[3] = {V1, V2, V3};
  // Serial.print("V1: ");
  // Serial.println(V1);
  // Serial.print("V2: ");
  // Serial.println(V2);
  // Serial.print("V3: ");
  // Serial.println(V3);

  // Calculate RPM
  for (int i=0; i<3; i++){
    currentCounts[i] = myEnc[i].read();
    positionChange[i] = currentCounts[i] - lastCount[i];
    rpm[i] = (positionChange[i] / 1300.0) * (60 * (1000.0 / 75));
    lastCount[i] = currentCounts[i];

  }

  //PID Control
  for (int i=0; i<3; i++){
    err[i] = sp[i] - rpm[i];
    integ[i] = integ[i] + (err[i]*0.075);   
    der[i] = (err[i]-prev_err[i])/0.075;

    pid[i] = (kp[i]*err[i]) + (ki[i]*integ[i]) + (kd[i]*der[i]);
    prev_err[i] = err[i];

    pid[i] = constrain(pid[i], -16383, 16383);

    // Serial.print("pid1: ");
    // Serial.println(pid[0]);
    // Serial.print("pid2: ");
    // Serial.println(pid[1]);
    // Serial.print("pid3: ");
    // Serial.println(pid[2]);

    Serial.print("sp1: ");
    Serial.print(sp[0]);
    Serial.print("sp2: ");
    Serial.print(sp[1]);
    Serial.print("sp3: ");
    Serial.print(sp[2]);
    Serial.print("  ");

    Serial.print(" rpm1: ");
    Serial.print(rpm[0]);
    Serial.print(" rpm2: ");
    Serial.print(rpm[1]);
    Serial.print(" rpm3: ");
    Serial.print(rpm[2]);
    Serial.println();
  }

  // Set motor speeds based on calculated velocities
  runMotor(PWM[0], DIR[0], pid[0]);
  runMotor(PWM[1], DIR[1], pid[1]);
  runMotor(PWM[2], DIR[2], pid[2]);

  // delay(200);  // Small delay for stability

  // unsigned long currentTime = micros();
  // unsigned long time = currentTime-startTime;
  // Serial.println(time);
  
}

void runMotor(int EN, int IN, float speed) {
  int pwmValue = abs(speed);
  // int pwmValue = constrain(abs(speed),0,200);
  // int pwmValue = map(abs(speed), 0, 127, 0, 16383);

  if (speed > 0) {      // to check direction: if +ve - HIGH, else LOW
    digitalWrite(IN, HIGH);
  } else if (speed < 0) {
    digitalWrite(IN, LOW);
  } else {
    pwmValue = 0;
  }
  analogWrite(EN, pwmValue);
}

void setup() {
  Serial.begin(9600);

  // Motor control pins setup
  for (int i = 0; i < 3; i++) {
    pinMode(PWM[i], OUTPUT);
    pinMode(DIR[i], OUTPUT);
}

  // Initialize motor to stop
  for (int i = 0; i < 3; i++) {
    analogWrite(PWM[i], 0);
    digitalWrite(DIR[i], LOW);
}

  analogWriteResolution(14);
  analogWriteFrequency(0, 9000);

 // myusb.begin();
  delay(2000);

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
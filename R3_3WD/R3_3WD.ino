#include <Encoder.h>
#include <IntervalTimer.h> 
#include "USBHost_t36.h"

Encoder myEnc[3] = { Encoder(17, 16), Encoder(14, 15), Encoder(23, 22)};

//PS4 connection 
USBHost myusb;   // initializes and manages the USB host port, enabling Teensy to detect and commuincate with USB bluetooth dongle
USBHIDParser hid1(myusb);  //works behind the scenes to parse HID data that comes from the PS4 controller, such as joystick movements and button presses.
JoystickController joystick(myusb);
// BluetoothController bluet(myusb, true, "0000"); 
BluetoothController bluet(myusb);   // Version does pairing to device

//coordinates of joystick (x,y -> right joystick; leftX -> left joystick)
int x, y, leftX;  
int PWM[3] = {3,5,9};
int DIR[3] = {2,4,8};

long currentCounts[3] = {0,0,0};
volatile long lastCount[3] = {0,0,0};      
volatile double rpm[3] = {0,0,0};          // Stores the calculated RPM
long positionChange[3] = {0,0,0};

//pid constants
float kp[3] = {0.0, 0.0, 0.0};
float ki[3] = {0.0, 0.0, 0.0};
float kd[3] = {0.0, 0.0, 0.0}; 

volatile float sp[3]={0,0,0};
float pid[3] = {0.0, 0.0, 0.0};
float err[3] = {0.0, 0.0, 0.0};
float prev_err[3] = {0.0, 0.0, 0.0};
float integ[3] = {0.0, 0.0, 0.0};
float der[3] = {0.0, 0.0, 0.0}; 

float max_rpm = 500;

IntervalTimer timer; // Timer object for periodic execution

void input() {
  if (Serial.available() > 0) {
    String input = Serial.readString();       // 1,009.000,165.000,000.500
    int i = input.substring(0,1).toInt();
    kp[i-1] = input.substring(2,9).toFloat();
    ki[i-1] = input.substring(10,17).toFloat();
    kd[i-1] = input.substring(18,24).toFloat();

  }
}


void calculatePID() {
  // unsigned long startTime = micros();

  input();

  myusb.Task();   // Handle USB host tasks

  if (joystick.available()) {
    // if(joystick.getButtons()){
    //   // Serial.println("value");

    // }

    // Left Stick values (axes 0 and 1)
    int leftStickX = joystick.getAxis(0);
    leftX = map(leftStickX, 0, 255, -100, 100);

    // Right Stick values (axes 2 and 5)
    int rightStickX = joystick.getAxis(2);
    x = map(rightStickX, 0, 255, -100, 100);

    int rightStickY = joystick.getAxis(5);
    y = map(rightStickY, 0, 255, 100, -100);


    // round off 
    // x = round(x/10)*10;
    // y = round(y/10)*10;
    // leftX = round(leftX/10)*10;

    //to ignore small joystick values
    if (abs(x) < 5) x = 0;
    if (abs(y) < 5) y = 0;
    if (abs(leftX) < 5) leftX = 0;

    // Serial.printf(" x:%d\n",x);
    // Serial.printf(" y:%d",y);
    // Serial.printf(" left x:%d",leftX);
  }
  else{
  //  Serial.print("no value");
    // delay(500);
  }

  // Serial.print("x: ");
  // Serial.println(x);
  // Serial.print("y: ");
  // Serial.println(y);
  // Serial.print("leftx: ");
  // Serial.println(leftX);


  sp[0] = ((x) * (-0.67) + (y) * 0 + (leftX) * (-0.33));        
  sp[1] = ((x) * (0.33) + (y) * (-0.57) + (leftX) * (-0.33)); 
  sp[2] = ((x) * (0.33) + (y) * (0.57) + (leftX) * (-0.33)); 
  
  //  Serial.printf(" V1:%d",V1);
  //   Serial.printf(" V2:%d",V2);
  //   Serial.printf(" V3:%d\n",V3);


  // Serial.print("V1: ");
  // Serial.println(V1);
  // Serial.print("V2: ");
  // Serial.println(V2);
  // Serial.print("V3: ");
  // Serial.println(V3);

  sp[0] = map(sp[0], -72, 72, -max_rpm, max_rpm);
  sp[1] = map(sp[1], -72, 72, -max_rpm, max_rpm);
  sp[2] = map(sp[2], -72, 72, -max_rpm, max_rpm);

  Serial.printf(" sp1:%0.2f", sp[0]);
  Serial.printf(" sp2:%0.2f", sp[1]);
  Serial.printf(" sp3:%0.2f", sp[2]);

  // float sp[3];// = {V1, V2, V3};
  // sp[0]=V1;
  // sp[1]=V2;
  // sp[2]=V3;

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
  Serial.printf(" rpm1:%f", rpm[0]);
  Serial.printf(" rpm2:%f", rpm[1]);
  Serial.printf(" rpm3:%f\n", rpm[2]);

  //PID Control
  for (int i=0; i<3; i++){
    err[i] = sp[i] - rpm[i];
    integ[i] = integ[i] + (err[i]*0.075);   
    der[i] = (err[i]-prev_err[i])/0.075;

    pid[i] = (kp[i]*err[i]) + (ki[i]*integ[i]) + (kd[i]*der[i]);
    prev_err[i] = err[i];

    pid[i] = constrain(pid[i], -8191, 8191);

  //  Serial.printf(" V1:%d",V1);
  //   Serial.printf(" V2:%d",V2);
  //   Serial.printf(" V3:%d\n",V3);

   //  Serial.printf(" V1:%d",V1);
  //   Serial.printf(" V2:%d",V2);
  //   Serial.printf(" V3:%d\n",V3);
  

  // Serial.printf(" sp1:%0.2f", sp[0]);
  // Serial.printf(" sp2:%0.2f", sp[1]);
  // Serial.printf(" sp3:%0.2f\n", sp[2]);
  // Serial.printf(" rpm1:%f", rpm[0]);
  // Serial.printf(" rpm2:%f", rpm[1]);
  // Serial.printf(" rpm3:%f\n", rpm[2]);

    // Serial.print("pid1: ");
    // Serial.println(pid[0]);
    // Serial.print("pid2: ");
    // Serial.println(pid[1]);
    // Serial.print("pid3: ");
    // Serial.println(pid[2]);
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

  analogWriteResolution(13);
  analogWriteFrequency(0, 18000);

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
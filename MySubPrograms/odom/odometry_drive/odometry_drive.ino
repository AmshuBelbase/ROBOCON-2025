#include <Encoder.h>
#include "Wire.h"
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <IntervalTimer.h> 
#include "USBHost_t36.h"

Encoder myEnc[3] = {Encoder(28, 27), Encoder(12, 11), Encoder(9, 8)};

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire2); // 0x28 is the default I2C address

Encoder encoder1(18, 19); // Pins for encoder 1
Encoder encoder2(20, 21); // Pins for encoder 2

// Encoder encoder1(22,23);// Pins for encoder 1
// Encoder encoder2(20,21);// Pins for encoder 2

volatile int currentTick1 = 0; // Position for encoder 1
volatile int currentTick2 = 0; // Position for encoder 2

volatile int oldTick1 = 0; 
volatile int oldTick2 = 0; 

volatile int deltaTick1 = 0; 
volatile int deltaTick2 = 0; 

volatile float globalX = 0; 
volatile float globalY = 0; 

volatile float speedX=0;
volatile float speedY=0;

volatile float deltaX = 0; 
volatile float deltaY = 0;

volatile float totalX = 0; 
volatile float totalY = 0;

volatile float globalTheta=0;
volatile float oldTheta=0;
volatile float deltaTheta=0;
volatile float virtual_global_Theta=0;

const float wheelDiameter = 5.8; 
const int encoderResolutionX = 2300;
const int encoderResolutionY = 2300;
const float CY= (PI * wheelDiameter / encoderResolutionY );
const float CX= (PI * wheelDiameter / encoderResolutionX );

const float L = 24.5;
const int B = -19.5;

//PS4 connection 
USBHost myusb;   // initializes and manages the USB host port, enabling Teensy to detect and commuincate with USB bluetooth dongle
USBHIDParser hid1(myusb);  //works behind the scenes to parse HID data that comes from the PS4 controller, such as joystick movements and button presses.
JoystickController joystick(myusb);
BluetoothController bluet(myusb, true, "0000"); 
// BluetoothController bluet(myusb);   // Version does pairing to device

//coordinates of joystick (x,y -> right joystick; leftX -> left joystick)
int x, y, leftX;  
int PWM[3] = {7, 1, 2};
int DIR[3] = {6, 0, 3};

long currentCounts[3] = {0,0,0};
volatile long lastCount[3] = {0,0,0};      
volatile double rpm[3] = {0,0,0};          // Stores the calculated RPM
long positionChange[3] = {0,0,0};

//pid constants
float kp[3] = {9.0, 9.0, 9.0};
float ki[3] = {165.0, 165.0, 165.0};
float kd[3] = {0.5, 0.5, 0.5}; 

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
  odometry();
  Serial.print(globalX);
  Serial.print(",");
  Serial.print(globalY);
  Serial.print(",");
  Serial.print(totalX);
  Serial.print(",");
  Serial.print(totalY);
  Serial.print(",");
  Serial.println(globalTheta);
  // unsigned long startTime = micros();

  // input();
// 
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
    x = map(rightStickX, 0, 255, -60, 60);

    int rightStickY = joystick.getAxis(5);
    y = map(rightStickY, 0, 255, -60, 60);


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


  // Calculate wheel speeds based on inverse kinematics
  // int V1 = ((x) * (-0.67) + (y) * 0 + (leftX) * (-0.33));        
  // int V2 = ((x) * (0.33) + (y) * (-0.565) + (leftX) * (-0.33)); 
  // int V3 = ((x) * (0.33) + (y) * (0.59) + (leftX) * (-0.33)); 

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

  // Serial.printf(" sp1:%0.2f", sp[0]);
  // Serial.printf(" sp2:%0.2f", sp[1]);
  // Serial.printf(" sp3:%0.2f", sp[2]);

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
  // Serial.printf(" rpm1:%f", rpm[0]);
  // Serial.printf(" rpm2:%f", rpm[1]);
  // Serial.printf(" rpm3:%f\n", rpm[2]);

  //PID Control
  for (int i=0; i<3; i++){
    err[i] = sp[i] - rpm[i];
    integ[i] = integ[i] + (err[i]*0.075);   
    der[i] = (err[i]-prev_err[i])/0.075;

    pid[i] = (kp[i]*err[i]) + (ki[i]*integ[i]) + (kd[i]*der[i]);
    prev_err[i] = err[i];

    pid[i] = constrain(pid[i], -16383, 16383);

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
  Wire2.begin();
  Serial.begin(115200);

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
  // delay(2000);

  //UART.setSerialPort(&Serial1);
  Serial.println("\n\nUSB Host Testing - Joystick Bluetooth");
  if (CrashReport) Serial.print(CrashReport);
  myusb.begin();
  myusb.Task();

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  timer.begin(calculatePID, 75000);
  // while (!Serial) delay(10); // Wait for Serial Monitor

  // Initialize BNO055
  if (!bno.begin()) {
    Serial.println("BNO055 not detected. Check connections!");
    while (1);
  }
  Serial.println("BNO055 detected!");

  // Optional: Configure to NDOF mode (for fused orientation data)
  bno.setExtCrystalUse(true);
}

void loop() {
  // odometry();
  // delay(10);
  // Serial.print(globalX);
  // Serial.print(",");
  // Serial.println(globalY);

}

void odometry() {
  sensors_event_t event;
  bno.getEvent(&event);



  // Save old tick values
  oldTick1 = currentTick1;
  oldTick2 = currentTick2;

  // Read encoder positions
  currentTick1 = encoder1.read();
  currentTick2 = encoder2.read();
  //Serial.println(currentTick1);
  //Serial.println(currentTick2);
  totalY = currentTick2;
  totalX = currentTick1;


  // Calculate change in encoder ticks
  deltaTick1 = currentTick1 - oldTick1;
  deltaTick2 = currentTick2 - oldTick2;

  // Save the previous virtual global theta
  oldTheta = virtual_global_Theta;

  // Get the new global theta from the sensor
  globalTheta = -event.orientation.x; // BNO055 provides the heading angle
  globalTheta+=360;
  
  //Serial.println(globalTheta);

  // Calculate the change in theta from the IMU
  deltaTheta = globalTheta - oldTheta;

  // Handle wrapping of deltaTheta within -180° to 180°
  if (deltaTheta > 180){
    deltaTheta -= 360;
    //deltaTheta=0-deltaTheta;
  }
  if (deltaTheta < -180) {
    deltaTheta += 360;
    //deltaTheta=0-deltaTheta;
  }


  // Update virtual global theta and normalize it
  virtual_global_Theta += deltaTheta;

  // Convert the virtual global theta to radian
  float ThetaRad = virtual_global_Theta * PI / 180;

  // Compute local displacements
  deltaX = (deltaTick1 * CX) - (L * deltaTheta * PI / 180);
  deltaY = (deltaTick2 * CY) - (B * deltaTheta * PI / 180);


  // Convert local displacements to global coordinates
  double cos_theta = cos(ThetaRad);
  double sin_theta = sin(ThetaRad);

  double delta_x_global = cos_theta * deltaX - sin_theta * deltaY;
  double delta_y_global = sin_theta * deltaX + cos_theta * deltaY;

  // Update global positions
  globalX += delta_x_global;
  globalY += delta_y_global;
  

}

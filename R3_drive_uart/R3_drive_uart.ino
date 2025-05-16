#include <Encoder.h>
#include <IntervalTimer.h> 
#include "USBHost_t36.h"
#include <VescUart.h>
VescUart UART;

Encoder myEnc[3] = {Encoder(22,23), Encoder(16,17), Encoder(15,14)};

//PS4 connection 
USBHost myusb;   // initializes and manages the USB host port, enabling Teensy to detect and commuincate with USB bluetooth dongle
USBHIDParser hid1(myusb);  //works behind the scenes to parse HID data that comes from the PS4 controller, such as joystick movements and button presses.
JoystickController joystick(myusb);
// BluetoothController bluet(myusb, true, "0000");     // Version does connecting to device
BluetoothController bluet(myusb);   // Version does pairing to device

//coordinates of joystick (x,y -> right joystick; leftX -> left joystick)
int x = 0, y = 0, leftX = 0;  
uint32_t buttons;
bool flag = 0;
bool flag_timer=0;
int lastTime=0;

int LPWM[3] = {29, 2, 9};   // PWM signals
int RPWM[3] = {5, 3, 8};
// int EN[3] = {16,20,17};    // direction


long currentCounts[3] = {0,0,0};
volatile long lastCount[3] = {0,0,0};      
volatile double rpm[3] = {0,0,0};          // Stores the calculated RPM
long positionChange[3] = {0,0,0};

//pid constants
float kp[3] = {8.0, 8.0, 8.0};
float ki[3] = {150.0, 150.0, 150.0};
float kd[3] = {0.5, 0.5, 0.5}; 

volatile float sp[3]={0,0,0};
float pid[3] = {0.0, 0.0, 0.0};
float err[3] = {0.0, 0.0, 0.0};
float prev_err[3] = {0.0, 0.0, 0.0};
float integ[3] = {0.0, 0.0, 0.0};
float der[3] = {0.0, 0.0, 0.0}; 

float max_rpm = 600;

IntervalTimer timer; // Timer object for periodic execution
IntervalTimer ps4_timer;
IntervalTimer bldc_timer;

// bldc
int rpm_bldc = 0;
bool flag_timer_bldc = 0;
int lastTime_bldc=0;

void input() {
  if (Serial.available() > 0) {
    String input = Serial.readString();       // 1,009.000,165.000,000.500
    int i = input.substring(0,1).toInt();
    kp[i-1] = input.substring(2,9).toFloat();
    ki[i-1] = input.substring(10,17).toFloat();
    kd[i-1] = input.substring(18,24).toFloat();
  }
}

uint32_t button=0;
uint32_t prev_button;

void ps4(){
  myusb.Task();
  if (joystick.available()) {
    // isJoystick = true;
    // for (uint8_t i = 0; i < 64; i++) {
    //   psAxis_prev[i] = psAxis[i];
    //   psAxis[i] = joystick1.getAxis(i);
    // }
  buttons = joystick.getButtons();
  button = buttons;
  if (button != prev_button){
    Serial8.write((uint8_t*)&button, sizeof(button));
    Serial.println(button);
    prev_button = button;
  }
  // Serial8.write((uint8_t*)&button, sizeof(button));
  // Serial.println(button);  
  }
}

void calculatePID() {
  // unsigned long startTime = micros();

  // input();

  myusb.Task();   // Handle USB host tasks

  if (joystick.available()) {

    // Left Stick values (axes 0 and 1)
    int leftStickX = joystick.getAxis(2);
    leftX = map(leftStickX, 0, 255, -100, 100);

    // Right Stick values (axes 2 and 5)
    int rightStickX = joystick.getAxis(0);
    x = map(rightStickX, 0, 255, -100, 100);

    int rightStickY = joystick.getAxis(1);
    y = map(rightStickY, 0, 255, 100, -100);

    buttons = joystick.getButtons();

    if (buttons == 16 && (flag == 0 && flag_timer == 1)){
      max_rpm = 600;
      digitalWrite(13, HIGH);
      flag = 1;
      flag_timer = 0;
      Serial.println("600RPM");
    }
    else if (buttons == 16 && (flag == 1 && flag_timer == 1)){
      max_rpm = 300;
      digitalWrite(13, LOW);
      flag = 0;
      flag_timer = 0;
    Serial.println("300RPM");

    }
          // Serial.println(lastTime);

    // Serial.print(flag_timer);
    if(millis()-lastTime>=1000)
    {
      flag_timer = 1;
      lastTime=millis();
      
    }


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


  // Calculate wheel speeds based on inverse kinematics
  // int V1 = ((x) * (-0.67) + (y) * 0 + (leftX) * (-0.33));        
  // int V2 = ((x) * (0.33) + (y) * (-0.565) + (leftX) * (-0.33)); 
  // int V3 = ((x) * (0.33) + (y) * (0.59) + (leftX) * (-0.33)); 

  sp[0] = ((x) * (-0.67) + (y) * 0 + (leftX) * (-0.16));        
  sp[1] = ((x) * (0.33) + (y) * (-0.555) + (leftX) * (-0.16)); 
  sp[2] = ((x) * (0.33) + (y) * (0.57) + (leftX) * (-0.16)); 
  
  // Serial.printf(" sp1:%0.2f", sp[0]);
  // Serial.printf(" sp2:%0.2f", sp[1]);
  // Serial.printf(" sp3:%0.2f", sp[2]);

  sp[0] = map(sp[0], -72, 72, -max_rpm, max_rpm);
  sp[1] = map(sp[1], -72, 72, -max_rpm, max_rpm);
  sp[2] = map(sp[2], -72, 72, -max_rpm, max_rpm);

  // Serial.printf(" sp1:%0.2f", sp[0]);
  // Serial.printf(" sp2:%0.2f", sp[1]);
  // Serial.printf(" sp3:%0.2f", sp[2]);

  // Calculate RPM
  for (int i=0; i<3; i++){
    currentCounts[i] = myEnc[i].read();
    positionChange[i] = currentCounts[i] - lastCount[i];
    rpm[i] = (positionChange[i] / 700.0) * (60 * (1000.0 / 75));
    lastCount[i] = currentCounts[i];

  }
  // Serial.printf(" rpm1:%f", rpm[0]);
  // Serial.printf(" rpm2:%f", rpm[1]);
  // Serial.printf(" rpm3:%f\n", rpm[2]);
  // Serial.printf("MAX_RPM:%f\n", max_rpm);

  //PID Control
  for (int i=0; i<3; i++){
    err[i] = sp[i] - rpm[i];
    integ[i] = integ[i] + (err[i]*0.075);   
    der[i] = (err[i]-prev_err[i])/0.075;

    pid[i] = (kp[i]*err[i]) + (ki[i]*integ[i]) + (kd[i]*der[i]);
    prev_err[i] = err[i];

    pid[i] = constrain(pid[i], -16383, 16383);
  

  // Serial.printf(" sp1:%0.2f", sp[0]);
  // Serial.printf(" sp2:%0.2f", sp[1]);
  // Serial.printf(" sp3:%0.2f\n", sp[2]);
  // Serial.printf(" rpm1:%f", rpm[0]);
  // Serial.printf(" rpm2:%f", rpm[1]);
  // Serial.printf(" rpm3:%f\n", rpm[2]);

  }

  // Set motor speeds based on calculated velocities
  runMotor(RPWM[0], LPWM[0], pid[0]);
  runMotor(RPWM[1], LPWM[1], pid[1]);
  runMotor(RPWM[2], LPWM[2], pid[2]);

  // delay(200);  // Small delay for stability

  // unsigned long currentTime = micros();
  // unsigned long time = currentTime-startTime;
  // Serial.println(time);
  
}

void runMotor(int RPWM, int LPWM, float speed) {
  int pwmValue = int(abs(speed));
  // int pwmValue = constrain(abs(speed),0,200);
  // int pwmValue = map(abs(speed), 0, 127, 0, 16383);

  if (speed > 0) {      // to check direction: if +ve - HIGH, else LOW
    // digitalWrite(EN, HIGH);

    analogWrite(RPWM, pwmValue);
    analogWrite(LPWM, 0);
  } else if (speed < 0) {
    // digitalWrite(EN, LOW);

    analogWrite(RPWM, 0);
    analogWrite(LPWM, pwmValue);
  } else {
    // digitalWrite(EN, LOW);
    
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
  }
}

void setup() {
  Serial.begin(9600);
  Serial8.begin(115200);    //teensy
  Serial7.begin(115200);    //bldc
  UART.setSerialPort(&Serial7);

  // Motor control pins setup
  for (int i = 0; i < 3; i++) {
    pinMode(RPWM[i], OUTPUT);
    pinMode(LPWM[i], OUTPUT);
    // pinMode(EN[i], OUTPUT);
}

  // Initialize motor to stop
  for (int i = 0; i < 3; i++) {
    analogWrite(RPWM[i], 0);
    analogWrite(LPWM[i], 0);
    // digitalWrite(EN[i], LOW);
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
  ps4_timer.begin(ps4, 10000);
  bldc_timer.begin(bldc, 10000);
  timer.priority(2);
  bldc_timer.priority(1);
  ps4_timer.priority(0);
}

void bldc() {
  if (buttons == 65536 && flag_timer_bldc==1){       // up
    buttons = 0;
    if (rpm_bldc ==0){
      rpm_bldc = 500;
    }
    // Serial.println("rpm up");
    rpm_bldc += 500;
    UART.setRPM(rpm_bldc);
    flag_timer_bldc = 0;
  }
  else if (buttons == 65552 && flag_timer_bldc==1){       // up
    buttons = 0;
    if (rpm_bldc ==0){
      rpm_bldc = 500;
    }
    // Serial.println("rpm_bldc up");
    rpm_bldc += 100;
    UART.setRPM(rpm_bldc);
    flag_timer_bldc = 0;
  }
  else if (buttons == 262144 && flag_timer_bldc==1){      // down
    buttons = 0;
    rpm_bldc -= 500;
    UART.setRPM(rpm_bldc);
    flag_timer_bldc = 0;
  }
  else if (buttons == 262160 && flag_timer_bldc==1){      // down
    buttons = 0;
    rpm_bldc -= 100;
    UART.setRPM(rpm_bldc);
    flag_timer_bldc = 0;
  }

  if(millis()-lastTime_bldc>=1000)
  {
    for(int i=0;i<20;i++)
    {flag_timer_bldc=1;
    }
    lastTime_bldc=millis();
    // Serial.println(flag_timer_bldc);
  }
  

}
void loop(){

}
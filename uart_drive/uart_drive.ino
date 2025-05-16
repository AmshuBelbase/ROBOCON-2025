#include "USBHost_t36.h"
#include <Encoder.h>

IntervalTimer pid_timer;
IntervalTimer ps4Timer;

// drive
// Encoder myEnc[3] = { Encoder(36, 33), Encoder(38, 37), Encoder(40, 39) };
Encoder myEnc[3] = { Encoder(22, 23), Encoder(20, 21), Encoder(18, 17) };

//PS4 connection 
USBHost myusb;   // initializes and manages the USB host port, enabling Teensy to detect and commuincate with USB bluetooth dongle
USBHIDParser hid1(myusb);  //works behind the scenes to parse HID data that comes from the PS4 controller, such as joystick movements and button presses.
JoystickController joystick1(myusb);
// BluetoothController bluet(myusb, true, "0000"); 
BluetoothController bluet(myusb);   // Version does pairing to device
uint32_t buttons;
bool isJoystick = 0;
bool flag = 0;
bool flag_timer=0;
int lastTime=0;

//coordinates of joystick (x,y -> right joystick; leftX -> left joystick)
int x, y, leftX;  
// int PWM[3] = { 4, 5, 3 };
// int DIR[3] = { 6, 7, 2 };
int PWM[3] = { 4, 6, 10 };
int DIR[3] = { 3, 5, 11 };


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

//////// drive


void setup() {

  Serial.begin(9600);
  Serial8.begin(115200);

  // teensy led
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);


  // drive
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

    //UART.setSerialPort(&Serial1);
            Serial.println("\n\nUSB Host Testing - Joystick Bluetooth");
            if (CrashReport) Serial.print(CrashReport);
            myusb.begin();
            myusb.Task();




  analogWriteResolution(14);
  analogWriteFrequency(0, 9000);

  pid_timer.priority(2);
  ps4Timer.begin(ps4, 10000);
  ps4Timer.priority(0);
  pid_timer.begin(drive_pid, 75000);


}
uint32_t button=0;
uint32_t prev_button;

void ps4(){
  myusb.Task();
  if (joystick1.available()) {
    isJoystick = true;
    // for (uint8_t i = 0; i < 64; i++) {
    //   psAxis_prev[i] = psAxis[i];
    //   psAxis[i] = joystick1.getAxis(i);
    // }
  buttons = joystick1.getButtons();
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

// drive
void drive_pid() {

  // input();
  myusb.Task();   // Handle USB host tasks

  if (isJoystick == true) {

    // Left Stick values (axes 0 and 1)
    int leftStickX = joystick1.getAxis(0);
    leftX = map(leftStickX, 0, 255, -100, 100);

    // Right Stick values (axes 2 and 5)
    int rightStickX = joystick1.getAxis(2);
    x = map(rightStickX, 0, 255, -100, 100);

    int rightStickY = joystick1.getAxis(5);
    y = map(rightStickY, 0, 255, 100, -100);

    // round off 
    // x = round(x/10)*10;
    // y = round(y/10)*10;
    // leftX = round(leftX/10)*10;
    if (buttons == 32 && (flag == 0 && flag_timer == 1)){
      max_rpm = 500;
      digitalWrite(13, HIGH);
      flag = 1;
      flag_timer = 0;
      Serial.println("500RPM");
    }
    else if (buttons == 32 && (flag == 1 && flag_timer == 1)){
      max_rpm = 250;
      digitalWrite(13, LOW);
      flag = 0;
      flag_timer = 0;
    Serial.println("250RPM");

    }
          // Serial.println(lastTime);

    // Serial.print(flag_timer);
    if(millis()-lastTime>=1000)
    {
      flag_timer = 1;
      lastTime=millis();
      
    }

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

  sp[0] = ((x) * (-0.67) + (y) * 0 + (leftX) * (-0.33));        
  sp[1] = ((x) * (0.33) + (y) * (-0.57) + (leftX) * (-0.33)); 
  sp[2] = ((x) * (0.33) + (y) * (0.57) + (leftX) * (-0.33)); 

  sp[0] = map(sp[0], -72, 72, -max_rpm, max_rpm);
  sp[1] = map(sp[1], -72, 72, -max_rpm, max_rpm);
  sp[2] = map(sp[2], -72, 72, -max_rpm, max_rpm);

  Serial.printf(" sp1:%0.2f", sp[0]);
  Serial.printf(" sp2:%0.2f", sp[1]);
  Serial.printf(" sp3:%0.2f", sp[2]);

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

    digitalWrite(DIR[i], (pid[i] <= 0 ? LOW : HIGH));
    analogWrite(PWM[i], abs(pid[i]));

    pid[i] = constrain(pid[i], -16383, 16383);

  }

}
//////drive




void loop() {

}

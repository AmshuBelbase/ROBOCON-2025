#define TX_PIN 35 // Teensy 4.1 Tx pin (Serial8)
#define RX_PIN 34 // Teensy 4.1 Rx pin (Serial8)

#include "USBHost_t36.h"
#include <Encoder.h>

IntervalTimer pidTimer;
IntervalTimer ps4Timer;


int pwm_pin[3] = { 4, 5, 3 };
int dir_pin[3] = { 6, 7, 2 };


Encoder m[3] = { Encoder(36, 33), Encoder(38, 37), Encoder(40, 39) };

volatile float rpm_rt[3] = { 0, 0, 0 };

int res = pow(2, 14) - 1;

int duty_cycle = 100;                           //in percentage
int max_pwm = (int)(duty_cycle / 100.0 * res);  //6v--250rpm
int max_rpm = 500;


//PS
USBHost myusb;
JoystickController joystick1(myusb);
// BluetoothController bluet(myusb, true, "0000");   // Version does pairing to device
BluetoothController bluet(myusb);  // version assumes it already was paireduint32_t buttons_prev = 0;
uint32_t buttons;

int psAxis[64];
int psAxis_prev[64];
bool first_joystick_message = true;
//..PS4


void setup() {
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);


  Serial.begin(200000);
  Serial8.begin(115200);

  Serial.println("\n\nUSB Host Testing - Joystick Bluetooth");
  if (CrashReport) Serial.print(CrashReport);
  myusb.begin();

    for (int i = 0; i < 3; i++) 
  {
    analogWriteFrequency(pwm_pin[i], 9000);
    pinMode(dir_pin[i], OUTPUT);
  }
  analogWriteResolution(14);


  // pidTimer.begin(pid, 75000);
  ps4Timer.begin(ps4, 10000);
  ps4Timer.priority(0);
  pidTimer.priority(1);


}


bool isJoystick=false;


volatile long oldPosition[3] = { 0, 0, 0 };
int ledState = LOW;
volatile unsigned long count[3] = { -999, -999, -999 };  // use volatile for shared variables
volatile long newPosition[3] = { 0, 0, 0 };

volatile int pwm_pid[] = { 0, 0, 0 };
volatile float rpm_sp[] = { 0, 0, 0 };


volatile float kp[] = { 09.0, 09.0, 09.0 };
volatile float ki[] = { 165.0, 165.0, 165.0 };
volatile float kd[] = { 00.50, 00.50, 00.50 };

float error[] = { 0, 0, 0 };
float eInt[] = { 0, 0, 0 };
float eDer[] = { 0, 0, 0 };
float lastError[] = { 0, 0, 0 };


void pid() {



  //~~~~~~~~~~~
  for (int i = 0; i < 3; i++) {
    newPosition[i] = m[i].read();
    count[i] = abs(newPosition[i] - oldPosition[i]);
    // count=newPosition<oldPosition?-count:count;
    rpm_rt[i] = count[i] / 1300.0 * 600 * 4 / 3;
    rpm_rt[i] *= newPosition[i] < oldPosition[i] ? -1 : 1;

    Serial.printf("RPM_output(motor: %d):%0.2f ", i + 1, rpm_rt[i]);
    count[i] = 0;
    oldPosition[i] = newPosition[i];
  }
  Serial.printf("\n");
  //~~this block of code is to calculate current RPM

  //~~~~~~~~~~~~~~~~~~~~~
  if (isJoystick == true) {
    int psAxisX = 0;
    int psAxisY = 0;
    int w = 0;
    if (psAxis[0] < 125)
      psAxisX = map(psAxis[0], 125, 0, 0, -255);

    else if (psAxis[0] > 135)
      psAxisX = map(psAxis[0], 135, 255, 0, 255);
    else
      psAxisX = 0;

    if (psAxis[1] > 135)
      psAxisY = map(psAxis[1], 135, 255, 0, 255);

    else if (psAxis[1] < 125)
      psAxisY = map(psAxis[1], 125, 0, 0, -255);
    else
      psAxisY = 0;
    if (psAxis[2] > 135)
      w = map(psAxis[2], 135, 255, 0, 255);

    else if (psAxis[2] < 125)
      w = map(psAxis[2], 125, 0, 0, -255);

    else
      w = 0;

    int y = psAxisY;
    int x = psAxisX;

    Serial.print(x);
    Serial.print("   ok ");
    Serial.print(y);
    Serial.println();
    rpm_sp[0] = map(x + w, -175, 175, max_rpm, -max_rpm);
    rpm_sp[1] = map(-0.5 * x - 0.866 * y + w, -175, 175, max_rpm, -max_rpm);
    rpm_sp[2] = map(-0.5 * x + 0.866 * y + w, -175, 175, max_rpm, -max_rpm);

    for (int i = 0; i < 3; i++) {
      Serial.printf("RPM_%d_input:%0.2f  ", i + 1, rpm_sp[i]);
    }
    //~~this block of code is to take the input from the ps4 controller
  }



  //~~~~~~~~~

  for (int i = 0; i < 3; i++) {
    error[i] = rpm_sp[i] - rpm_rt[i];
    eDer[i] = (error[i] - lastError[i]) / 0.075;
    eInt[i] = eInt[i] + error[i] * 0.075;

    pwm_pid[i] = int(kp[i] * error[i] + ki[i] * eInt[i] + kd[i] * eDer[i]);
    //Serial.printf("pwm_pid:%d ",pwm_pid[i]);
    //pwm_pid[i]=map(pwm_pid[i],-16383,16383,-pwm_18,pwm_18);
    //Serial.printf("pwm_pid:%d \n",pwm_pid[i]);

    digitalWrite(dir_pin[i], (pwm_pid[i] <= 0 ? LOW : HIGH));
    analogWrite(pwm_pin[i], abs(pwm_pid[i]));

    lastError[i] = error[i];
    // Serial.printf("RPM_%d_input:%0.2f  ",i+1, rpm_sp[i]);
  }

  //~~This block of code does the PID of the drive
}

int lasttime=0;
uint32_t button=0;
uint32_t prev_button;

void ps4(){
  myusb.Task();
  if (joystick1.available()) {
    isJoystick = true;

    for (uint8_t i = 0; i < 64; i++) {
      psAxis_prev[i] = psAxis[i];
      psAxis[i] = joystick1.getAxis(i);
      // Serial8.write((uint8_t*)psAxis, sizeof(psAxis));
    }
    if(millis()-lasttime>=1000){
      lasttime=millis();
      Serial8.write(0xAA);
      Serial8.write((uint8_t*)psAxis, sizeof(psAxis));
    }
  buttons = joystick1.getButtons();
  button = buttons;
  if (button != prev_button){
    Serial8.write((uint8_t*)&button, sizeof(button));
    Serial.printf("button: %d\n",button);
    prev_button = button;
  }
  // Serial8.write((uint8_t*)&button, sizeof(button));
  // Serial.println(button);  
  }

}

void loop() {

  // myusb.Task();
  // if (joystick1.available()) {
  //   isJoystick = true;
  //   for (uint8_t i = 0; i < 64; i++) {
  //     psAxis_prev[i] = psAxis[i];
  //     psAxis[i] = joystick1.getAxis(i);
  //   }
  //   buttons = joystick1.getButtons();
  //   Serial.println(buttons);
  //   button=buttons;

  // if(buttons == 4){
  //   Serial8.println("dribble");
  // }
    // Send array
  // if(millis()-lastTime>=100)
  // {lastTime=millis();
  // // Serial8.write(0xAA);
  //   // Serial8.write((uint8_t*)psAxis, sizeof(psAxis));

  //   // Send variable
  //   if (button != prev_button){
  //     Serial8.write((uint8_t*)&button, sizeof(button));
  //     Serial.println(button);
  //     prev_button = button;
  //   }
  

  // //   Serial.println(button); // Debugging
  // }
  // }

  //  else {
  //   isJoystick = false;
  // }


}
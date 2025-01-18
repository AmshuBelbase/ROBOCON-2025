#include "USBHost_t36.h"
#include <Encoder.h>
IntervalTimer rpm_timer;
IntervalTimer ps4_timer;
IntervalTimer pid_timer;
IntervalTimer pos_pid_timer;


int pwm_pin[3] = { 3, 7, 5};
int dir_pin[3] = { 2, 6, 4};


Encoder m[3] = { Encoder(28, 27), Encoder(31, 30), Encoder(12, 11) };
Encoder enc1(9,8);

int shooting_angle=0;

int shooter_rotation_pwm=1;
int shooter_rotation_dir=0;
float shooter_rotation_cpr=17500;
float kP=50,kI=1,kD=1;

float ap=0.0;
float sp_angle=0.0;

volatile float rpm_rt[3] = { 0, 0, 0 };

int res = pow(2, 14) - 1;
int duty_cycle = 100;                            //in percentage
int max_pwm = (int)(duty_cycle / 100.0 * res);  //6v--250rpm
int max_rpm = 200;//hello.....

//int res=pow(2,14)-1;
//int duty_cycle=25;//in percentage
//int max_pwm =(int)(duty_cycle/100.0*res); //6v--250rpm
// void drive(int pwm_val, int pwmPin, int dirPin) {
//   digitalWrite(dirPin, (pwm_val <= 0 ? LOW : HIGH));
//   // Serial.printf("Dir: %d\n", (pwm_val <= 0 ? LOW : HIGH));
//   //Serial.printf("PWM: %d \n", abs(pwm_val),);
//   analogWrite(pwmPin, abs(pwm_val));
//   // Serial.println("Drive");
// }//HelloWorld pi5

USBHost myusb;
JoystickController joystick1(myusb);
// BluetoothController bluet(myusb, true, "0000");   // Version does pairing to device
BluetoothController bluet(myusb);   // version assumes it already was paireduint32_t buttons_prev = 0;
uint32_t buttons;

int psAxis[64];
int psAxis_prev[64];
bool first_joystick_message = true;

// int m2_pwm = 3;
// int m2_dir = 2;


void setup() {

  //Serial.begin(200000);
  // while (!Serial) ; // wait for Arduino Serial Monitor

  Serial.println("\n\nUSB Host Testing - Joystick Bluetooth");
  if (CrashReport) Serial.print(CrashReport);
  myusb.begin();

  for (int i = 0; i < 3; i++) {
    analogWriteFrequency(pwm_pin[i], 9000);
    pinMode(dir_pin[i], OUTPUT);
  }

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  // pinMode(2,OUTPUT);
  pinMode(shooter_rotation_pwm,OUTPUT);
  pinMode(shooter_rotation_dir,OUTPUT);
  Serial.begin(200000);
  ps4_timer.priority(0);
  rpm_timer.priority(1);
  pid_timer.priority(2);
  pos_pid_timer.priority(3);

  //const_timer.priority(3);
  ps4_timer.begin(ps4_input, 75000);
  rpm_timer.begin(calc_rpm, 75000);
  pid_timer.begin(pid, 75000);
  pos_pid_timer.begin(pos_pid,10000);

  //const_timer.begin(input,75000);
  // calcRpm() to run every 0.1 seconds....us
  analogWriteResolution(14);

  // analogWriteFrequency(m4_pwm, 9000);

  //  analogWrite(m4_pwm,6000);
  //  Serial.println("move");
  //  analogWrite(m4_dir,res);
}
volatile long oldPosition[3] = { 0, 0, 0};
int ledState = LOW;
volatile unsigned long count[3] = { -999, -999, -999 };  // use volatile for shared variables
volatile long newPosition[3] = { 0, 0, 0};


void setPosition(int speed)
{
//Serial.printf("speed_dir: %d",speed<0?LOW:HIGH);

digitalWrite(shooter_rotation_dir,speed<0?LOW:HIGH);

//speed=abs(speed)>=1920?1920:abs(speed);
//Serial.printf("speed_drive: %d",abs(speed));
analogWrite(shooter_rotation_pwm,abs(speed));



}

float sp=0;
float err=0,der=0,integ=0;
float speed_m=0;
volatile float last_err=0;

int lastTime=0;


void pos_pid()
{
ap=enc1.read();
sp=sp_angle*shooter_rotation_cpr/360.0;

err=sp-ap;
der=(err-last_err)/0.01;
integ+=(err-last_err)*0.01;
last_err=err;

speed_m=kP*err+kI*integ+kD*der;

// Serial.printf("speed: %f   ",speed_m);
// Serial.printf("Kp: %f.   Ki: %f.    Kd: %f   ",kp,ki,kd);
// Serial.print("Sp:");
// Serial.print(shooting_angle);
// Serial.print("     Ap:");
// Serial.println(int((ap*360.0/shooter_rotation_cpr)/6.25));
      // Serial.println(enc1.read());
setPosition(int(speed_m));

}

void calc_rpm() {
  for (int i = 0; i < 3; i++) {
    newPosition[i] = m[i].read();
    count[i] = abs(newPosition[i] - oldPosition[i]);
    // count=newPosition<oldPosition?-count:count;
    rpm_rt[i] = count[i] / 1300.0 * 600 * 4 / 3;
    rpm_rt[i] *= newPosition[i] < oldPosition[i] ? -1 : 1;

   Serial.printf("RPM_output(motor: %d):%0.2f ",i+1, rpm_rt[i]);
    count[i] = 0;
    oldPosition[i] = newPosition[i];
  }
  Serial.printf("\n");
}
volatile int pwm_pid[] = {0,0,0};
volatile float rpm_sp[] = {0,0,0};
void ps4_input() {
  myusb.Task();
  if (joystick1.available()) {
    for (uint8_t i = 0; i < 64; i++) {
      psAxis_prev[i] = psAxis[i];
      psAxis[i] = joystick1.getAxis(i);
    }

    buttons = joystick1.getButtons();
/*
 s1=int(1*x)
         s2=int(-0.508*x-0.88*y)
         s3=int(-0.5045*x+0.8725*y)

*/
int psAxisX=0;
int psAxisY=0;
int w= 0;
if(psAxis[0]<125)
         psAxisX=map(psAxis[0],125,0,0,-255);
      
      else if(psAxis[0]>135)
        psAxisX=map(psAxis[0],135,255,0,255);
      else
         psAxisX=0;
      
      if(psAxis[1]>135)
        psAxisY=map(psAxis[1],135,255,0,255);
      
      else if(psAxis[1]<125)
         psAxisY=map(psAxis[1],125,0,0,-255);
      else
         psAxisY=0;
      if(psAxis[2]>135)
        w=map(psAxis[2],135,255,0,255);
      
      else if(psAxis[2]<125)
         w=map(psAxis[2],125,0,0,-255);
      else
         w=0;
    int y = psAxisY;
    int x= psAxisX;

    
    rpm_sp[0] = map(x+w,-175,175,max_rpm,-max_rpm);
    rpm_sp[1] = map(-0.5*x-0.866*y+w,-175,175,max_rpm,-max_rpm);
    rpm_sp[2] = map(-0.5*x+0.866*y+w,-175,175,max_rpm,-max_rpm);
   // rpm_sp[3] = map(x-y+w,-255,255,max_rpm,-max_rpm);
    //Serial.printf("time: %d\n ",millis());
    for(int i=0;i<3;i++)
    Serial.printf("RPM_%d_input:%0.2f  ",i+1, rpm_sp[i]);
  }
}
volatile float kp[] = {09.0,09.0,09.0};
volatile float ki[] = {165.0,165.0,165.0};
volatile float kd[] = {00.50,00.50,00.50};

float error[]={0,0,0};
float eInt[]={0,0,0};
float eDer[]={0,0,0}; 
float lastError[]={0,0,0};
//void input()
//{
//
//
//  }
void pid() {
  //Serial.println(rpm);
  for(int i=0;i<3;i++){
  error[i] = rpm_sp[i] - rpm_rt[i];
  eDer[i] = (error[i] - lastError[i]) / 0.075;
  eInt[i] = eInt[i] + error[i] * 0.075;

  pwm_pid[i] = int(kp[i] * error[i] + ki[i] * eInt[i] + kd[i] * eDer[i]);
  //Serial.printf("pwm_pid:%d ",pwm_pid);
  digitalWrite(dir_pin[i], (pwm_pid[i] <= 0 ? LOW : HIGH));
  analogWrite(pwm_pin[i], abs(pwm_pid[i]));

  lastError[i] = error[i];
  }
}

float rpmCopy = 0;
// The main program will print the blink count
// to the Arduino Serial Monitor
//int lastTime = 0;
int ind=-1;
void loop() {

  //  if (millis() - lastTime > 1000) {
  //    noInterrupts();
  //    if (Serial.available() > 0) {
  //      String input = Serial.readString();//1,030.000,145.000,000.500
  //      Serial.println(input.length());
  //      if (input.length() <= 100) {
  //        ind=(input.substring(0, 1)).toInt()-1;
  //        kp[ind] = (input.substring(2, 9)).toFloat();
  //        ki[ind] = (input.substring(10, 17)).toFloat();
  //        kd[ind] = (input.substring(18, 25)).toFloat();
  //        Serial.printf("%d   %f %f %f\n", ind,kp[ind], ki[ind], kd[ind]);
  //      }
  //    } 
  //      interrupts();
  //    lastTime = millis();
  //  }


  //ps4_input();
 // delay(1000);
if (millis() - lastTime > 1000) {
      noInterrupts();

     if (Serial.available() > 0) {
       String input = Serial.readString();//0050,0001,0001,030
      // Serial.println(input.length());
       if (input.length() <= 20) {
         //char sign=(input.substring(0, 1)).charAt(0);
         kP=(input.substring(0,4)).toFloat();
         kI=(input.substring(5,9)).toFloat();
         kD=(input.substring(10,14)).toFloat();
         shooting_angle=input.substring(15).toInt();
         sp_angle=int((input.substring(15)).toInt()*6.25);
       }
     } 
        interrupts();
      lastTime = millis();
    }

}

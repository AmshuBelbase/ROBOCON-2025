#include "USBHost_t36.h"
#include <Encoder.h>
#include <VescUart.h>
#include <Servo.h>
#include <Pixy2SPI_SS.h>
Pixy2SPI_SS pixy;

volatile int pwm_18=0;
bool isJoystick = 0;
VescUart UART;

// IntervalTimer rpm_timer;
// IntervalTimer ps4_timer;
IntervalTimer pid_timer;
IntervalTimer pos_pid_timer;
IntervalTimer feed_pid_timer;

// IntervalTimer serial_input_timer;

//driblling 
int roller1_pin=29;//left
int roller1_spd=27;

// int roller2_pin=17;//right
// int roller2_spd=19;

int piston1=1;
int piston2=0;

int ball_push=3;
int ball_pull=2;

Servo servo_left;
Servo servo_right;
//dribbling

// drive pinouts 
int pwm_pin[3] = {22, 19, 18};
int dir_pin[3] = {20, 17, 16};


Encoder m[3] = { Encoder(14,15), Encoder(40,41), Encoder(38,39) };
Encoder myEnc1(6,7);   // feeding 

//pid constants
float Kp = 50.0;   //kp=35
float Ki = 0.5;   //ki=0.5
float Kd = 0.3;   //kd=0.3

double prevError1 = 0;          // double error1, error2, prevError1 = 0, prevError2 = 0;
int sp1 = 0;
float err1 = 0;
float integ1=0.0;
float der1=0.0;
float pid1 = 0.0;
long currentCounts1 = 0;

//Encoder enc1(9,8);

volatile int feeder_pwm=28;
volatile int feeder_dir=26;

int shooting_angle=0;

int shooter_rotation_pwm=28;
int shooter_rotation_dir=26;
float shooter_rotation_cpr=17500;
volatile float kP = 8.5, kI = 0.15, kD = 0.15;

volatile int error_offset = 0;
volatile int bldc_rpm=0;
volatile int last_bldc_rpm=0;
float ap=0.0;
float sp_angle=0.0;

volatile float rpm_rt[3] = { 0, 0, 0 };

int res = pow(2, 14) - 1;

int duty_cycle = 100;                            //in percentage
int max_pwm = (int)(duty_cycle / 100.0 * res);  //6v--250rpm
int max_rpm = 500;

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

pixy.init();
// pixy.setLamp(1,1);


Serial.begin(200000);
Serial1.begin(115200);
servo_left.attach(37);
servo_right.attach(36);
// while (!Serial) ; // wait for Arduino Serial Monitor
UART.setSerialPort(&Serial1);
Serial.println("\n\nUSB Host Testing - Joystick Bluetooth");
if (CrashReport) Serial.print(CrashReport);
myusb.begin();
myusb.Task();
// if (joystick1.available()) {}

for (int i = 0; i < 3; i++) {
analogWriteFrequency(pwm_pin[i], 9000);
pinMode(dir_pin[i], OUTPUT);
}

pinMode(13, OUTPUT);
digitalWrite(13, HIGH);
// pinMode(2,OUTPUT);
pinMode(shooter_rotation_pwm,OUTPUT);
pinMode(shooter_rotation_dir,OUTPUT);

pinMode(feeder_dir,OUTPUT);


pid_timer.priority(1);
feed_pid_timer.priority(0);
// Serial.begin(200000);
// ps4_timer.priority(4);
// rpm_timer.priority(3);
//pid_timer.priority(1);
//pos_pid_timer.priority(0);
// serial_input_timer.priority(1);

//const_timer.priority(3);
//ps4_timer.begin(ps4_input, 75000);
//rpm_timer.begin(calc_rpm, 75000);
pid_timer.begin(pid, 75000);
// serial_input_timer.begin(serial_input,10000);
// pos_pid_timer.begin(pos_pid, 10000);

//const_timer.begin(input,75000);
// calcRpm() to run every 0.1 seconds....us
analogWriteResolution(14);

// analogWriteFrequency(m4_pwm, 9000);

//  analogWrite(m4_pwm,6000);
//  Serial.println("move");
//  analogWrite(m4_dir,res);


//DRIBBLING
// pinMode(13,OUTPUT);
// digitalWrite(13,HIGH);

pinMode(roller1_pin, OUTPUT);
pinMode(roller1_spd, OUTPUT);

// pinMode(roller2_pin, OUTPUT);
// pinMode(roller2_spd, OUTPUT);

pinMode(ball_push,OUTPUT);
pinMode(ball_pull,OUTPUT);

digitalWrite(ball_push,HIGH);
digitalWrite(ball_pull,LOW);
delay(1000);
digitalWrite(ball_push,HIGH);
digitalWrite(ball_pull,HIGH);


pinMode(piston1,OUTPUT);
pinMode(piston2,OUTPUT);

digitalWrite(piston1,HIGH);
digitalWrite(piston2,LOW);
delay(1000);
digitalWrite(piston1,HIGH);
digitalWrite(piston2,HIGH);

for(int angle=30;angle>=0;angle--){
//if(angle<125)
servo_right.write(180-angle);
//Serial.println(180-angle);

servo_left.write(angle);
Serial.println(angle);
delay(15);
}//dribbling



}
volatile long oldPosition[3] = { 0, 0, 0};
int ledState = LOW;
volatile unsigned long count[3] = { -999, -999, -999 };  // use volatile for shared variables
volatile long newPosition[3] = { 0, 0, 0};


void setPosition(int speed)
{//Serial.println("set pos");
//Serial.printf("speed_dir: %d",speed<0?LOW:HIGH);
digitalWrite(shooter_rotation_dir,speed<0?LOW:HIGH);
//speed=abs(speed)>=1920?1920:abs(speed);
//Serial.printf("speed_drive: %d",abs(speed));
analogWrite(shooter_rotation_pwm,abs(speed));
}

float sp = 0;
float err = 0, der = 0, integ = 0;
float speed_m = 0;
volatile float last_err = 0;
volatile int lastTime=0;
char inputBuffer[20];
int inputIndex = 0;
volatile bool track=false;
//int lastTime=0;

// void serial_input() {



// }

void pos_pid() {
// ap = enc1.read();
// sp = sp_angle * shooter_rotation_cpr / 360.0;
//~~~~~~~~~
while (Serial.available() > 0) {
char received = Serial.read();

if (received == '\n') {  // End of input
inputBuffer[inputIndex] = '\0';  // Null-terminate

// Split the string using ',' as a delimiter
char *part1 = strtok(inputBuffer, ",");
char *part2 = strtok(NULL, ",");

if (part1 != NULL) error_offset = atoi(part1);
if (part2 != NULL) bldc_rpm = atoi(part2);

memset(inputBuffer, 0, sizeof(inputBuffer));  // Clear buffer
inputIndex = 0;  // Reset index

//       Print values for debugging
// Serial.print("Value 1: ");
// Serial.println(error_offset);
// Serial.print("Value 2: ");
// Serial.println(bldc_rpm);

} else if (inputIndex < sizeof(inputBuffer) - 1) {  // Prevent overflow
inputBuffer[inputIndex++] = received;
}
}  if(abs(last_bldc_rpm-bldc_rpm)>20)
{
UART.setRPM(bldc_rpm*7);
last_bldc_rpm=bldc_rpm;
}

// else if(received == 'n')
// {
//   error_offset=0;
// }
// else if(received == 'o')
// {
//   error_offset=0;
// }
// Serial.print("The error offset: ");
// Serial.println(error_offset);

//~~this block takes Serial input from python script
err = error_offset;
der = (err - last_err) / 0.01;
integ += (err - last_err) * 0.01;
last_err = err;

speed_m = kP * err + kI * integ + kD * der;
speed_m=constrain(speed_m,-4370,4370);

// Serial.printf("speed: %f   ",speed_m);
// Serial.printf("Kp: %f.   Ki: %f.    Kd: %f   ",kp,ki,kd);
// Serial.println(speed_m);
// Serial.print(shooting_angle);
// Serial.print("     Ap:");
// Serial.println(int((ap * 360.0 / shooter_rotation_cpr) / 6.25));
// Serial.println(enc1.read());
setPosition(int(speed_m));
// if(track)
// {  digitalWrite(13,LOW);
//   track=false;
// }
// else
//   {
//     digitalWrite(13,HIGH);
//     track=true;
//   }
}

// void calc_rpm() {

// }
volatile int pwm_pid[] = {0,0,0};
volatile float rpm_sp[] = {0,0,0};

// void ps4_input() {

// //void input()
// //{
// //
// //
// //  }}
// }
volatile float kp[] = {09.0,09.0,09.0};
volatile float ki[] = {165.0,165.0,165.0};
volatile float kd[] = {00.50,00.50,00.50};

float error[]={0,0,0};
float eInt[]={0,0,0};
float eDer[]={0,0,0}; 
float lastError[]={0,0,0};
void pid() {

//   if ( UART.getVescValues() ) {

// pwm_18=18.0/UART.data.inpVoltage*16383;
// // Serial.print("Voltage: ");
// // Serial.println(UART.data.inpVoltage);
// }


//~~~~~~~~~~~
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
//~~this block of code is to calculate current RPM

//~~~~~~~~~~~~~~~~~~~~~
// myusb.Task();
//if (joystick1.available()) {
  if(isJoystick == true){
// for (uint8_t i = 0; i < 64; i++) {
// psAxis_prev[i] = psAxis[i];
// psAxis[i] = joystick1.getAxis(i);
// }
// buttons = joystick1.getButtons();
//  Serial.println(buttons);
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
  
Serial.print(x);
Serial.print("   ok ");
Serial.print(y);
Serial.println();
rpm_sp[0] = map(x+w,-175,175,max_rpm,-max_rpm);
rpm_sp[1] = map(-0.5*x-0.866*y+w,-175,175,max_rpm,-max_rpm);
rpm_sp[2] = map(-0.5*x+0.866*y+w,-175,175,max_rpm,-max_rpm);
// rpm_sp[3] = map(x-y+w,-255,255,max_rpm,-max_rpm);
//   Serial.printf("time: %d\n ",millis());
for(int i=0;i<3;i++){
 Serial.printf("RPM_%d_input:%0.2f  ",i+1, rpm_sp[i]);
}
//~~this block of code is to take the input from the ps4 controller
  }



//~~~~~~~~~
//Serial.println(rpm);
// Serial.println("pid");
for(int i=0;i<3;i++){
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




//DRIBBLING
void servomotion(int start_angle, int end_angle){
if(start_angle<end_angle){ 
for(int angle=start_angle;angle<=end_angle;angle++){
// if(angle<125)
servo_right.write(180-angle);

servo_left.write(angle);
delay(15);
}
}
else if(start_angle>end_angle){
for(int angle=start_angle;angle>=end_angle;angle--){
// if(angle<125)
servo_right.write(180-angle);
//Serial.println(180-angle);

servo_left.write(angle);
//Serial.println(angle);
delay(15);
}
}

}

void rollers(){
digitalWrite(roller1_pin,HIGH);
analogWrite(roller1_spd,255*64);

//  digitalWrite(roller2_pin,HIGH);
//  analogWrite(roller2_spd,255); 

}

void stoprollers(){
digitalWrite(roller1_pin,LOW);
analogWrite(roller1_spd,0);

//  digitalWrite(roller2_pin,LOW);
//  analogWrite(roller2_spd,0); 


}

void dcv_control(){
digitalWrite(ball_push,LOW);
digitalWrite(ball_pull,HIGH);
delay(500);

// digitalWrite(ball_push,LOW);
// digitalWrite(ball_pull,LOW);
// delay(1000);

digitalWrite(ball_push,HIGH);
digitalWrite(ball_pull,LOW);
// delay(500);
// digitalWrite(ball_push,LOW);
// digitalWrite(ball_pull,HIGH);
delay(500);
digitalWrite(ball_push,HIGH);
digitalWrite(ball_pull,HIGH);  

}
// void tof(){
//   if(tflI2C.getData(tfDist, tfAddr)){
//        Serial.println(String(tfDist)+" cm / " + String(tfDist/2.54)+" inches");
//     // }
//   //delay(50);
//   if(tfDist<25){
//   digitalWrite(piston1,HIGH);
//   digitalWrite(piston2,LOW);
//   delay(1000);
//   // digitalWrite(piston1,LOW);
//   // digitalWrite(piston2,LOW);
//   // delay(500);
//   digitalWrite(piston1,LOW);
//   digitalWrite(piston2,HIGH);


//   }}
//}
void function(){
digitalWrite(piston1,LOW);
digitalWrite(piston2,HIGH);
delay(1000);
// digitalWrite(piston1,LOW);
// digitalWrite(piston2,LOW);
// delay(500);
digitalWrite(piston1,HIGH);
digitalWrite(piston2,LOW);
delay(500);
digitalWrite(piston1,HIGH);
digitalWrite(piston2,HIGH);


}
/////DRIBBLING



float rpmCopy = 0;
// The main program will print the blink count
// to the Arduino Serial Monitor
//int lastTime = 0;
int ind=-1;
volatile float rt_volt=0.0;

// FEEDING
void calcPID() {
  // unsigned long startTime = micros();
  // int sp = 0;

  // input();
  Serial.print("sp: ");
  Serial.println(sp1);
 
  currentCounts1 = myEnc1.read();
  // long currentCounts2 = myEnc2.read();

  // PID Control (for M1)
  err1 = sp1 - currentCounts1;
  integ1 = integ1 + (err1*0.075);
  der1 = (err1 - prevError1)/0.075;

  pid1 = (Kp*err1) + (Ki*integ1) + (Kd*der1);
  prevError1 = err1;
  Serial.printf("error: %f", err1);
  Serial.printf("pid: %f\n", pid1);

  // PID Control (for M2)
  // float err2 = sp - currentCounts2;
  // integ2 = integ2 + (err2*0.075);
  // der2 = (err2 - prevError2)/0.075;

  // pid2 = (kp*err2) + (ki*integ2) + (kd*der2);
  // prevError2 = err2;
  // digitalWrite(feeder_dir, (pid1 <= 0 ? LOW : HIGH));
  // analogWrite(feeder_pwm, abs(pid1));

  runMotor(feeder_pwm, feeder_dir, pid1);

  // Serial.print("sp:");
  // Serial.print(sp);
  // Serial.print("ticks:");
  // Serial.println(myEnc1.read());

}
// void feed_up() {
//     calcPID(-3580);
// }
// void feed_down() {
//     calcPID(0);         // if ticks reset to 0, then angle = +3580
// }

void runMotor(int motorPWM, int motorDir, float speed) {
  int pwm = abs(speed);
  pwm = constrain(pwm, 0, 16383);
  if (speed > 0) {      //to check direction: if +ve - HIGH, else LOW

    digitalWrite(motorDir, HIGH);
  } else if (speed < 0) {
    digitalWrite(motorDir, LOW);
    speed = -speed;
  } else {
    pwm = 0;
  }
  analogWrite(motorPWM, pwm);
}

void loop() {

  myusb.Task();
if (joystick1.available()) {
  isJoystick=true;
for (uint8_t i = 0; i < 64; i++) {
psAxis_prev[i] = psAxis[i];
psAxis[i] = joystick1.getAxis(i);
}
buttons = joystick1.getButtons();}
else
{
  isJoystick=false;
}
//int i; 
// grab blocks!
//pixy.ccc.getBlocks();

// If there are detect blocks, print them!
// if (pixy.ccc.numBlocks)
// {
//   Serial.print("Detected ");
//   Serial.println(pixy.ccc.numBlocks);
//   for (i=0; i<pixy.ccc.numBlocks; i++)
//   {
//     Serial.print("  block ");
//     Serial.print(i);
//     Serial.println(": ");
//     //pixy.ccc.blocks[i].print();
//     Serial.println(pixy.ccc.blocks[i].m_height);
//     Serial.println(pixy.ccc.blocks[i].m_width);
//     Serial.println(pixy.ccc.blocks[i].m_age);

//   }
// }  
//noInterrupts();

//interrupts();
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
//    lastTime = millis(
//Serial.printf("button: %d\n",buttons);
if(buttons==2)
{
  // {servomotion(0,150);
  sp1 = 3550;
  feed_pid_timer.begin(calcPID, 75000);
  //  if (err1 == 0){
  //   feed_pid_timer.end();
  // }

  Serial.println("Cross");
  delay(2000);

  // ---------------SHOOTING-----------------------

  sp1 = 200;
  feed_pid_timer.begin(calcPID, 75000);
  // if (err1 == 0){
  //   feed_pid_timer.end();
  // }
}
else if(buttons == 8)
{
// Serial.println("Triangle");
// servomotion(150,0);
}
else if(buttons ==4)
{
//  Serial.println("Stopping PID Timer");

pid_timer.begin(pid, 100000000);  // Stop the interrupt timer
//Serial.println("PID Timer Stopped");
Serial.println("DRIBBLING");
// rollers();
// dcv_control();
servomotion(0,110);
delay(1000);
dcv_control();
delay(300);
rollers();
delay(600);
pixy.setLamp(1,1);
while(true)
{
pixy.ccc.getBlocks();

if (pixy.ccc.numBlocks)
{function();
Serial.println("Detected");
pixy.setLamp(0,0);
break;}

// if (joystick1.getButtons()){
//   break;
// }
}
//function();
delay(200);
servomotion(110,0);
stoprollers();
delay(2000);

// Serial.println("Restarting PID Timer");
pid_timer.begin(pid, 75000);
// Serial.println("PID Timer Restarted");

// // feeding 
// Serial.println("FEEDING");
// sp1 = 3500;
// feed_pid_timer.begin(calcPID, 75000);
// // if (err1 == 0){
// //   feed_pid_timer.end();
// // }

}

//  }


//ps4_input();
//  // delay(1000);
// if (millis() - lastTime > 1000) {
//       noInterrupts();

//      if (Serial.available() > 0) {
//        String input = Serial.readString();//0050,0001,0001,030
//       // Serial.println(input.length());
//        if (input.length() <= 20) {
//          //char sign=(input.substring(0, 1)).charAt(0);
//          kP=(input.substring(0,4)).toFloat();
//          kI=(input.substring(5,9)).toFloat();
//          kD=(input.substring(10,14)).toFloat();
//          shooting_angle=input.substring(15).toInt();
//          sp_angle=int((input.substring(15)).toInt()*6.25);
//        }
//      } 
//         interrupts();
//       lastTime = millis();
//     }

}
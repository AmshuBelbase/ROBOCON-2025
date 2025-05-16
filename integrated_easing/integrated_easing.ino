#include "USBHost_t36.h"
#include <Encoder.h>
#include <VescUart.h>
#include <Servo.h>
#include <Pixy2I2C.h>
Pixy2I2C pixy;

volatile int pwm_18 = 0;
bool isJoystick = 0;
VescUart UART;
// constrain(speed_feed,-16319*0.9,16319*0.9);

IntervalTimer pid_timer;

IntervalTimer feed_pid_timer;

// IntervalTimer serial_input_timer;
int pwm_pin[3] = { 22, 19, 18 };
int dir_pin[3] = { 20, 17, 16 };


Encoder m[3] = { Encoder(14, 15), Encoder(40, 41), Encoder(38, 39) };


IntervalTimer pos_pid_timer;
IntervalTimer feeder_pos_pid_timer;
Encoder encL(10, 9);
Encoder encR(7, 6);

Encoder encFeed(11, 12); 

int feeder_pwm=22;      //22
int feeder_dir=20;      //20

int feeder_cpr=538;
float ap_count_feeder=0;
float sp_angle_feeder=0;
float sp_count_feeder=0;


int shooter_rotation_pwmR = 19;
int shooter_rotation_dirR = 17;


int shooter_rotation_pwmL = 18;
int shooter_rotation_dirL = 16;

float shooter_rotation_cpr = 538;

float kP=150,kI=5,kD=1;


int shooting_angleL=0;
int shooting_angleR=0;

float apL=0.0;
float sp_angleL=0.0;

float apR=0.0;
float sp_angleR=0.0;



// //driblling
int roller1_pin = 21;  //left
int roller1_spd = 23;

// int roller2_pin = 21;  //right
// int roller2_spd = 23;

int piston1 = 1;
int piston2 = 0;

int ball_push = 3;
int ball_pull = 2;

int dribbling_arm_out=32;
int dribbling_arm_in=30;

//Servo servo_left;
//Servo servo_right;
Servo sr;
Servo sl;
unsigned long previousMillis = 0;
int startPos, endPos, steps = 100;
unsigned long duration;
int currentStep = 0;
bool isMoving = false;


Servo servo_stopper;
//dribbling

// drive pinouts

// Encoder myEnc1(6, 7);  // feeding





//Encoder enc1(9,8);

// volatile int feeder_pwm = 28;
// volatile int feeder_dir = 26;



volatile float rpm_rt[3] = { 0, 0, 0 };

int res = pow(2, 14) - 1;

int duty_cycle = 100;                           //in percentage
int max_pwm = (int)(duty_cycle / 100.0 * res);  //6v--250rpm
int max_rpm = 500;



//PS4
USBHost myusb;
JoystickController joystick1(myusb);
BluetoothController bluet(myusb, true, "0000");   // Version does pairing to device
BluetoothController bluet(myusb);  // version assumes it already was paireduint32_t buttons_prev = 0;
uint32_t buttons;

int psAxis[64];
int psAxis_prev[64];
bool first_joystick_message = true;
//..PS4




void setup() {

  pixy.init();
  pixy.setLamp(1,1);//to turn on pixy led

  Serial.begin(200000);
  Serial7.begin(115200);
  //servo_left.attach(37);
  //servo_right.attach(36);

 // servo_right.write(180-5);
 // servo_left.write(0+5);
  sr.attach(36);
  sl.attach(37);
  sr.write(180-15);
  sl.write(15);


  servo_stopper.attach(31);
  servo_stopper.write(0);
  // while (!Serial) ; // wait for Arduino Serial Monitor
  UART.setSerialPort(&Serial7);
  Serial.println("\n\nUSB Host Testing - Joystick Bluetooth");
  if (CrashReport) Serial.print(CrashReport);
  myusb.begin();
  myusb.Task();
  // if (joystick1.available()) {}

  for (int i = 0; i < 3; i++) 
  {
    analogWriteFrequency(pwm_pin[i], 9000);
    pinMode(dir_pin[i], OUTPUT);
  }

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  // pinMode(2,OUTPUT);

  pinMode(feeder_dir, OUTPUT);


  // pid_timer.priority(2);
  pos_pid_timer.priority(0);
  feed_pid_timer.priority(1);

  // pid_timer.begin(pid, 75000);
  pos_pid_timer.begin(pos_pid,10000);
  feed_pid_timer.begin(feed_pos_pid,10000);

  analogWriteResolution(14);
 pinMode(roller1_pin, OUTPUT);
  pinMode(roller1_spd, OUTPUT);



  pinMode(ball_push, OUTPUT);
  pinMode(ball_pull, OUTPUT);

  digitalWrite(ball_push, HIGH);
  digitalWrite(ball_pull, LOW);
  delay(1000);
  digitalWrite(ball_push, HIGH);
  digitalWrite(ball_pull, HIGH);


  pinMode(piston1, OUTPUT);
  pinMode(piston2, OUTPUT);

  digitalWrite(piston1, HIGH);
  digitalWrite(piston2, LOW);
  delay(1000);
  digitalWrite(piston1, HIGH);
  digitalWrite(piston2, HIGH);


  // for (int angle = 30; angle <= 45; angle++) {
  //   //if(angle<125)
  //   servo_right.write(180 - angle);
  //   //Serial.println(180-angle);

  //   servo_left.write(angle);
  //   Serial.println(angle);
  //   delay(15);
  // }  //dribbling


  pinMode(shooter_rotation_pwmL,OUTPUT);
  pinMode(shooter_rotation_dirL,OUTPUT);

  pinMode(shooter_rotation_pwmR,OUTPUT);
  pinMode(shooter_rotation_dirR,OUTPUT);

  pinMode(feeder_pwm,OUTPUT);
  pinMode(feeder_dir,OUTPUT);

  pinMode(dribbling_arm_out,OUTPUT);
  pinMode(dribbling_arm_in,OUTPUT);
  digitalWrite(dribbling_arm_out, HIGH);
  digitalWrite(dribbling_arm_in, LOW);
  delay(1000);
  digitalWrite(dribbling_arm_out, HIGH);
  digitalWrite(dribbling_arm_in, HIGH);
  // analogWriteResolution(14);



}
void setPosition(int pwm,int dir ,int speed)
{
  digitalWrite(dir,speed<0?LOW:HIGH);
  analogWrite(pwm,abs(speed));
}
float spL=0;
float errL=0,derL=0,integL=0;
float speed_mL=0;
volatile float last_errL=0;

float spR=0;
float errR=0,derR=0,integR=0;
float speed_mR=0;
volatile float last_errR=0;
void pos_pid()
{
apL=encL.read();
spL=sp_angleL*shooter_rotation_cpr/360.0;

errL=spL-apL;
derL=(errL-last_errL)/0.01;
integL=integL+(errL-last_errL)*0.01;
last_errL=errL;

speed_mL=kP*errL+kI*integL+kD*derL;



apR=encR.read();
spR=sp_angleR*shooter_rotation_cpr/360.0;

errR=spR-apR;
derR=(errR-last_errR)/0.01;
integR=integR+(errR-last_errR)*0.01;
last_errR=errR;

speed_mR=kP*errR+kI*integR+kD*derR;


constrain(speed_mR,-14361,14361);
constrain(speed_mL,-14361,14361);



// Serial.printf("speed: %f   ",speed_m);
// // Serial.printf("Kp: %f.   Ki: %f.    Kd: %f   ",kp,ki,kd);
// Serial.print("SpL:");
// Serial.print(sp_angleL);
// Serial.print("     ApL:");
// Serial.println(int((apL*360.0/shooter_rotation_cpr)));
//       // Serial.println(enc1.read());
setPosition(shooter_rotation_pwmL,shooter_rotation_dirL,int(speed_mL));

// Serial.print("SpR:");
// Serial.print(sp_angleR);
// Serial.print("     ApR:");
// Serial.println(int((apR*360.0/shooter_rotation_cpr)));
      // Serial.println(enc1.read());
setPosition(shooter_rotation_pwmR,shooter_rotation_dirR,int(speed_mR));



}
float err_feed=0,der_feed=0,integ_feed=0,lastError_feed=0;
float speed_feed=0;
void feed_pos_pid()
{

 /*int feeder_pwm=-1;
int feeder_dir=-2;

int feeder_cpr=538;
float ap_count_feeder=0;
float sp_angle_feeder=0;*/

ap_count_feeder=encFeed.read();
sp_count_feeder=sp_angle_feeder*feeder_cpr/360.0;

err_feed=sp_count_feeder-ap_count_feeder;
der_feed=(err_feed-lastError_feed)/0.01;
integ_feed=integ_feed+(err_feed-lastError_feed)*0.01;
lastError_feed=err_feed;

speed_feed=kP*err_feed+kI*integ_feed+kD*der_feed;
constrain(speed_feed,-14361,14361);
// Serial.printf("speed: %f   ",speed_m);
// Serial.printf("Kp: %f.   Ki: %f.    Kd: %f   ",kp,ki,kd);
// Serial.print("Sp_feed:");
// Serial.print(sp_angle_feed);
// Serial.print("     Ap_feed:");
// Serial.println(int((ap_angle_feed*360.0/shooter_rotation_cpr)));
// //       // Serial.println(enc1.read());
setPosition(feeder_pwm,feeder_dir,int(speed_feed));

}



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




//DRIBBLING
/*void servomotion(int start_angle, int end_angle) {
  if (start_angle < end_angle) {
    for (int angle = start_angle; angle <= end_angle; angle++) {
      // if(angle<125)
      servo_right.write(180 - angle);

      servo_left.write(angle-3);
      delay(15);
    }
  } else if (start_angle > end_angle) {
    for (int angle = start_angle; angle >= end_angle; angle--) {
      // if(angle<125)
      servo_right.write(180 - angle);
      //Serial.println(180-angle);

      servo_left.write(angle-3);
      //Serial.println(angle);
      delay(15);
    }
  }
}*/

float easeInOutCirc(float x) {
return x < 0.5 ? (1 - sqrt(1 - pow(2 * x, 2))) / 2 : (sqrt(1 - pow(-2 * x + 2, 2)) + 1) / 2;
}

void servomotion(int Spos, int Epos, int time) {
  startPos = Spos;
  endPos = Epos;
  duration = time;
  previousMillis = millis();  // Reset time
  currentStep = 0;
  isMoving = true;  // Start motion

  while(isMoving){
    updateServo();
  }
}

void updateServo() {
  if (!isMoving) return;

  unsigned long currentMillis = millis();
  unsigned long stepDuration = duration / steps;

  if (currentMillis - previousMillis >= stepDuration) {
    previousMillis = currentMillis;

    float progress = (float)currentStep / steps;
    float easedProgress = easeInOutCirc(progress);

    int servoPosL = startPos + round((endPos - startPos) * easedProgress);
    int servoPosR = 180 - (startPos + round((endPos - startPos) * easedProgress));

    sr.write(servoPosR);

    sl.write(servoPosL);

    Serial.print("sr: "); Serial.print(servoPosL);
    Serial.print(" | sl: "); Serial.println(servoPosR);

    currentStep++;
    if (currentStep >= steps) {
      isMoving = false;
      currentStep = 0;
    }
  }
}



void rollers() {
  digitalWrite(roller1_pin, LOW);
  analogWrite(roller1_spd, 255 * 64);

  //  digitalWrite(roller2_pin,HIGH);
  //  analogWrite(roller2_spd,255);
}

void stoprollers() {
  digitalWrite(roller1_pin, HIGH);
  analogWrite(roller1_spd, 0);

  //  digitalWrite(roller2_pin,LOW);
  //  analogWrite(roller2_spd,0);
}

void dcv_control() {
  digitalWrite(ball_push, LOW);
  digitalWrite(ball_pull, HIGH);
  delay(500);

  // digitalWrite(ball_push,LOW);
  // digitalWrite(ball_pull,LOW);
  // delay(1000);

  digitalWrite(ball_push, HIGH);
  digitalWrite(ball_pull, LOW);
  // delay(500);
  // digitalWrite(ball_push,LOW);
  // digitalWrite(ball_pull,HIGH);
  delay(500);
  digitalWrite(ball_push, HIGH);
  digitalWrite(ball_pull, HIGH);
}

void function() {
  digitalWrite(piston1, LOW);
  digitalWrite(piston2, HIGH);
  delay(1000);

  digitalWrite(piston1, HIGH);
  digitalWrite(piston2, LOW);
  delay(500);
  digitalWrite(piston1, HIGH);
  digitalWrite(piston2, HIGH);
}
/////DRIBBLING



float rpmCopy = 0;
int ind = -1;
volatile float rt_volt = 0.0;

// FEEDING
int flag_timer[20]={0};
int flag_rec=0;
int flag_feed=0;
int lastTime=0;
void loop() {
  // Serial.println("ok");
  // UART.setRPM(1500);

  //updateServo();

  myusb.Task();
  if (joystick1.available()) {
    isJoystick = true;
    for (uint8_t i = 0; i < 64; i++) {
      psAxis_prev[i] = psAxis[i];
      psAxis[i] = joystick1.getAxis(i);
    }
    buttons = joystick1.getButtons();
    // Serial.println(buttons);
  } else {
    isJoystick = false;
  }
  // Serial.println(isJoystick);
  if (buttons == 2 && flag_feed==0 && flag_timer[2]==1) {
    // {servomotion(0,150);
    // UART.setRPM(1500);
    Serial.println("Cross pressed");
    sp_angle_feeder=-3550;
    flag_feed=1;
    flag_timer[2]=0;
    }
    else if(buttons == 2 && flag_feed==1 && flag_timer[2]==1)
    {
    sp_angle_feeder=0;
    flag_feed=0;
    flag_timer[2]=0;
    // UART.setRPM(0);
    }
   else if (buttons == 8 && flag_rec==0 && flag_timer[8]==1) {

      sp_angleR=-4650;
      sp_angleL=4650;
      flag_rec=1;
      servo_stopper.write(0);
      Serial.println("Triangle pressed0");
      flag_timer[8]=0;    // servomotion(150,0);
  }
  else if(buttons == 8 && flag_rec==1 && flag_timer[8]==1)
  {
    sp_angleR=0;
      sp_angleL=0;
      flag_rec=0;
       Serial.println("Triangle pressed1");  
       flag_timer[8]=0;  // servomotion(150,0);
  }
  // else if(buttons ) 
  else if (buttons == 4) {
    //  Serial.println("Stopping PID Timer");
          Serial.println("Circle pressed");    // servomotion(150,0);

    servo_stopper.write(90); 
    feed_pid_timer.begin(feed_pos_pid,100000000);
    pos_pid_timer.begin(pos_pid,100000000);
    // pid_timer.begin(pid, 100000000);  // Stop the interrupt timer
    //Serial.println("PID Timer Stopped");
    Serial.println("DRIBBLING");
    // rollers();
    // dcv_control();
    digitalWrite(dribbling_arm_out,LOW);
    digitalWrite(dribbling_arm_in,HIGH);
    delay(500);
    digitalWrite(dribbling_arm_out,HIGH);
    digitalWrite(dribbling_arm_in,HIGH);
    
    servomotion(15,90,1000);
    //updateServo();
    delay(1000);
    dcv_control();
    delay(50);
    rollers();
    delay(600);
    pixy.setLamp(1, 1);
    while (true) {
      pixy.ccc.getBlocks();

      if (pixy.ccc.numBlocks) {
        function();
        Serial.println("Detected");
        pixy.setLamp(0, 0);
        break;
      }

      if (joystick1.getButtons()){
        Serial.println("Overridden");
        break;
      }
    }
    // function();
    delay(200);
    servomotion(90,15,1000);
    //updateServo();
    stoprollers();
    delay(500);
    servo_stopper.write(0); 
    digitalWrite(dribbling_arm_out,HIGH);
    digitalWrite(dribbling_arm_in,LOW);
    delay(500);
    digitalWrite(dribbling_arm_out,HIGH);
    digitalWrite(dribbling_arm_in,HIGH);
    // Serial.println("Restarting PID Timer");
    // pid_timer.begin(pid, 75000);
    // Serial.println("PID Timer Restarted");
    feed_pid_timer.begin(feed_pos_pid,10000);
    pos_pid_timer.begin(pos_pid,10000);
    // feeding
    // sp_angle_feeder=-3000;
    // feed_pid_timer.begin(calcPID, 10000);
    // if (err1 == 0) {
    //   feed_pid_timer.end();
    }
    if(millis()-lastTime>=1000)
    {
      for(int i=0;i<20;i++)
      {flag_timer[i]=1;
      }
      lastTime=millis();
    }
}
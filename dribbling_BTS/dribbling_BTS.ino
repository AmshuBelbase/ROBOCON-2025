#include "USBHost_t36.h"
#include <Encoder.h>
#include <VescUart.h>
#include <Servo.h>
#include <Pixy2I2C.h>
Pixy2I2C pixy;

VescUart UART;

IntervalTimer pid_timer;
IntervalTimer feed_pid_timer;
IntervalTimer rcv_pid_timer;

int flag_rec=0;
int flag_feed=0;

// drive
Encoder myEnc[3] = {Encoder(14,15), Encoder(40,41), Encoder(38,39)};

//PS4 connection 
USBHost myusb;   // initializes and manages the USB host port, enabling Teensy to detect and commuincate with USB bluetooth dongle
USBHIDParser hid1(myusb);  //works behind the scenes to parse HID data that comes from the PS4 controller, such as joystick movements and button presses.
JoystickController joystick1(myusb);
// BluetoothController bluet(myusb, true, "0000"); 
BluetoothController bluet(myusb);   // Version does pairing to device
uint32_t buttons;
bool isJoystick = 0;

//coordinates of joystick (x,y -> right joystick; leftX -> left joystick)
int x, y, leftX;  
int PWM[3] = {22, 19, 18};
int DIR[3] = {20, 17, 16};

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


//driblling 
int roller1_pin=23;//left
int roller1_spd=21;

int dribbling_arm_out=32;
int dribbling_arm_in=30;

int piston1=1;
int piston2=0;

int ball_push=3;
int ball_pull=2;

Servo servo_left;
Servo servo_right;
Servo servo_stop;
/////////dribbling


// feeding
Encoder myEnc_feed(6,7);   // feeding 
long currentCounts1 = 0;
volatile int feeder_pwm=22;
volatile int feeder_dir=20;

volatile float sp1;
float pid1 = 0.0;
float err1 = 0.0;
float prev_err1 = 0.0;
float integ1 = 0.0;
float der1 = 0.0; 
///////// feeding


// recieving 
Encoder myEnc_rcv[2] = {Encoder(6,7), Encoder(9,10)};

int rcv_pwm[2] = {19, 18};
int rcv_dir[2] = {17, 16};

//pid constants (rcv, feed)
float Kp = 50.0;   //kp=50
float Ki = 0.2;   //ki=0.2
float Kd = 0.3;   //kd=0.3

// long currentCounts[2] = {0,0};
long currentCounts2[2] = {0,0};

volatile float Rsp[2] = {0,0};
float Rpid[2] = {0.0, 0.0};
float Rerr[2] = {0.0, 0.0};
float Rprev_err[2] = {0.0, 0.0};
float Rinteg[2] = {0.0, 0.0};
float Rder[2] = {0.0, 0.0}; 
///////// recieving 


void setup() {

  Serial.begin(9600);
  Serial7.begin(115200);             // bldc
  UART.setSerialPort(&Serial7);

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



  // dribbling
  pixy.init();
  pixy.setLamp(1,1);//to turn on pixy led

  servo_left.attach(37);
  servo_right.attach(36); 
  servo_stop.attach(31);
  servo_stop.write(0);

  pinMode(roller1_pin, OUTPUT);
  pinMode(roller1_spd, OUTPUT);

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


  pinMode(dribbling_arm_out,OUTPUT);
  pinMode(dribbling_arm_in,OUTPUT);

  digitalWrite(dribbling_arm_out,HIGH);
  digitalWrite(dribbling_arm_in,LOW);
  delay(1000);
  digitalWrite(dribbling_arm_out,HIGH);
  digitalWrite(dribbling_arm_in,HIGH);


  // feeding
  pinMode(feeder_pwm, OUTPUT);
  pinMode(feeder_dir, OUTPUT);

  analogWrite(feeder_pwm, 0);
  digitalWrite(feeder_dir, LOW);



  // recieving
  for (int i = 0; i < 2; i++) {
    pinMode(rcv_pwm[i], OUTPUT);
    pinMode(rcv_dir[i], OUTPUT);
  }


  // Initialize motor to stop
  for (int i = 0; i < 2; i++) {
    analogWrite(rcv_pwm[i], 0);
    digitalWrite(rcv_dir[i], LOW);
  }



  analogWriteResolution(14);
  analogWriteFrequency(0, 9000);

  rcv_pid_timer.priority(0);
  feed_pid_timer.priority(1);
  pid_timer.priority(2);

  pid_timer.begin(drive_pid, 75000);
  feed_pid_timer.begin(feed_pid, 10000);
  rcv_pid_timer.begin(rcv_pid, 10000);

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

  // Serial.printf(" sp1:%0.2f", sp[0]);
  // Serial.printf(" sp2:%0.2f", sp[1]);
  // Serial.printf(" sp3:%0.2f", sp[2]);

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

    digitalWrite(DIR[i], (pid[i] <= 0 ? LOW : HIGH));
    analogWrite(PWM[i], abs(pid[i]));

    pid[i] = constrain(pid[i], -16383, 16383);

  }

}
//////drive

//DRIBBLING
void servomotion(int start_angle, int end_angle){
  if(start_angle<end_angle){ 
    for(int angle=start_angle;angle<=end_angle;angle++){
    // if(angle<125)
    servo_right.write(180-angle);

    servo_left.write(angle-3);
    delay(15);
    }
  }
  else if(start_angle>end_angle){
    for(int angle=start_angle;angle>=end_angle;angle--){
    // if(angle<125)
    servo_right.write(180-angle);
    //Serial.println(180-angle);
    servo_left.write(angle-3);
    //Serial.println(angle);
    delay(15);
    }
  }

}

void rollers(){
digitalWrite(roller1_pin,HIGH);
analogWrite(roller1_spd,255*64);

}

void stoprollers(){
digitalWrite(roller1_pin,LOW);
analogWrite(roller1_spd,0);

}

void dcv_control(){
digitalWrite(ball_push,LOW);
digitalWrite(ball_pull,HIGH);
delay(500);

digitalWrite(ball_push,HIGH);
digitalWrite(ball_pull,LOW);

delay(500);
digitalWrite(ball_push,HIGH);
digitalWrite(ball_pull,HIGH);  

}


void function(){
digitalWrite(piston1,LOW);
digitalWrite(piston2,HIGH);
delay(1000);

digitalWrite(piston1,HIGH);
digitalWrite(piston2,LOW);
delay(500);

digitalWrite(piston1,HIGH);
digitalWrite(piston2,HIGH);

}

void dribble_arm(){
digitalWrite(piston1,LOW);
digitalWrite(piston2,HIGH);
delay(1000);

digitalWrite(piston1,HIGH);
digitalWrite(piston2,LOW);
delay(500);

digitalWrite(piston1,HIGH);
digitalWrite(piston2,HIGH);

}

/////DRIBBLING


// FEEDING
void feed_pid() {

  // input();
  // Serial.print("sp: ");
  // Serial.println(sp1);
 
  currentCounts1 = myEnc_feed.read();

  err1 = sp1 - currentCounts1;
  integ1 = integ1 + (err1*0.075);
  der1 = (err1 - prev_err1)/0.075;

  pid1 = (Kp*err1) + (Ki*integ1) + (Kd*der1);
  prev_err1 = err1;
  // Serial.printf("error: %f", err1);
  // Serial.printf("pid: %f\n", pid1);

  digitalWrite(feeder_dir, (pid1 <= 0 ? LOW : HIGH));
  analogWrite(feeder_pwm, abs(pid1));

  // runMotor(feeder_pwm, feeder_dir, pid1);
}


// rcv
void rcv_pid() {

  for (int i = 0; i < 2; i++){
    currentCounts2[i] = myEnc_rcv[i].read();
  } 
  // Serial.printf("m1:%d", currentCounts[0]);
  // Serial.printf("m2:%d", currentCounts[1]);
  // currentCounts1 = myEnc1.read();
  // long currentCounts2 = myEnc2.read();

  for (int i=0; i<2; i++){
    Rerr[i] = Rsp[i] - currentCounts2[i];
    Rinteg[i] = Rinteg[i] + (Rerr[i]*0.010);   
    Rder[i] = (Rerr[i]-Rprev_err[i])/0.010;

    Rpid[i] = (Kp*Rerr[i]) + (Ki*Rinteg[i]) + (Kd*Rder[i]);
    prev_err[i] = err[i];

    Rpid[i] = constrain(pid[i], -16383, 16383);

    digitalWrite(rcv_dir[i], (Rpid[i] <= 0 ? LOW : HIGH));
    analogWrite(rcv_pwm[i], abs(Rpid[i]));

  }

  // Serial.printf(" sp1:%f", sp[0]);
  // Serial.printf(" ticks1:%d", myEnc[0].read());
  // Serial.printf(" sp2:%f", sp[1]);
  // Serial.printf(" ticks2:%d\n", myEnc[1].read());

}




void loop() {
  myusb.Task();
  if (joystick1.available()) {
    isJoystick = true;
    buttons = joystick1.getButtons();
    Serial.printf("button:%d",buttons);
  } else {
    isJoystick = false;
  }

  if (buttons == 2 && flag_feed==0) {
    UART.setRPM(1500);
    sp1=3400;
    flag_feed=1;
  }
  else if(buttons == 2 && flag_feed==1){
    sp1=0;
    flag_feed=0;
    UART.setRPM(0);
  }
  else if (buttons == 8 && flag_rec==0) {
    Rsp[0]=4650;
    Rsp[1]=-4650;
    flag_rec=1;
  }
  else if(buttons == 8 && flag_rec==1)
  {
    Rsp[0]=0;
    Rsp[1]=0;
    flag_rec=0;
  }
  else if (buttons == 4) {

    servo_stop.write(90); 
    feed_pid_timer.begin(feed_pid,100000000);
    rcv_pid_timer.begin(rcv_pid,100000000);
    pid_timer.begin(drive_pid, 100000000);  // Stop the interrupt timer

    Serial.println("DRIBBLING");
    // rollers();
    // dcv_control();
    digitalWrite(dribbling_arm_out,LOW);
    digitalWrite(dribbling_arm_in,HIGH);
    delay(500);
    digitalWrite(dribbling_arm_out,HIGH);
    digitalWrite(dribbling_arm_in,LOW);
    digitalWrite(dribbling_arm_out,HIGH);
    digitalWrite(dribbling_arm_in,HIGH);
    
    servomotion(0,88);
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
        break;
      }
    }
    function();
    delay(200);
    servomotion(88,0);
    stoprollers();
    delay(500);
    digitalWrite(dribbling_arm_out,LOW);
    digitalWrite(dribbling_arm_in,HIGH);
    delay(500);
    digitalWrite(dribbling_arm_out,HIGH);
    digitalWrite(dribbling_arm_in,HIGH);
    servo_stop.write(0);


    pid_timer.begin(drive_pid, 75000);
    feed_pid_timer.begin(feed_pid,10000);
    rcv_pid_timer.begin(rcv_pid,10000);
    // feeding
    // sp1 = 3000;
  
  }

}

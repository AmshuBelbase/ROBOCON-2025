#include "USBHost_t36.h"
#include <Servo.h>
#include <Pixy2I2C.h>
Pixy2I2C pixy;

int roller1_pin=11;//left
int roller1_spd=12;

int extend=29;
int retract=28;

int ball_push=27;
int ball_pull=26;

int dribble_arm_pwm[2] = {19, 18};
int dribble_arm_dir[2] = {17, 16};

bool limitSwitchState[2] = {false};       // 1        
int limitSwitch[2]={14,15};               // 0        

Servo servo_left;
Servo servo_right;
// Servo servo_stop;

uint32_t buttons;

//PS4 connection 
USBHost myusb;   // initializes and manages the USB host port, enabling Teensy to detect and commuincate with USB bluetooth dongle
USBHIDParser hid1(myusb);  //works behind the scenes to parse HID data that comes from the PS4 controller, such as joystick movements and button presses.
JoystickController joystick1(myusb);
// BluetoothController bluet(myusb, true, "0000");     // Version does connecting to device
BluetoothController bluet(myusb);   // Version does pairing to device


void setup() {
  Serial.begin(200000);

  pixy.init();
  pixy.setLamp(0,0);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  servo_left.attach(1);
  servo_right.attach(2); 
  // servo_stop.attach(3);
  // servo_stop.write(0);
  servo_left.write(15);
  servo_right.write(180-15);

  pinMode(roller1_pin, OUTPUT);
  pinMode(roller1_spd, OUTPUT);

  pinMode(ball_push,OUTPUT);
  pinMode(ball_pull,OUTPUT);

  digitalWrite(ball_push,HIGH);
  digitalWrite(ball_pull,LOW);
  delay(1000);
  digitalWrite(ball_push,LOW);
  digitalWrite(ball_pull,LOW);

  pinMode(extend,OUTPUT);
  pinMode(retract,OUTPUT);

  digitalWrite(extend,HIGH);
  digitalWrite(retract,LOW);
  delay(1000);
  digitalWrite(extend,LOW);
  digitalWrite(retract,LOW);

  for(int i=0;i<2;i++)
  {
    pinMode(limitSwitch[i],INPUT_PULLUP);
  }

  analogWriteResolution(14);
  analogWriteFrequency(0, 9000);

}

void loop() {
  for (int i = 0; i < 4; i++){
    limitSwitchState[i] = digitalRead(limitSwitch[i]);

  }

  myusb.Task();
  if (joystick1.available()) {
    buttons = joystick1.getButtons();
  }
  
  if (buttons == 4) {
    // servo_stop.write(90); 

    Serial.println("DRIBBLING");

    // digitalWrite(dribbling_arm_out,LOW);
    // digitalWrite(dribbling_arm_in,HIGH);
    // delay(500);
    // // digitalWrite(dribbling_arm_out,LOW);
    // // digitalWrite(dribbling_arm_in,HIGH);
    // digitalWrite(dribbling_arm_out,LOW);
    // digitalWrite(dribbling_arm_in,LOW);
    // dribble_arm_out();
    // delay(200);
    
    servomotion(15,90);
    delay(1000);
    piston1();
    delay(50);
    rollers();
    delay(600);
    pixy.setLamp(1, 1);
    while (true) {
      pixy.ccc.getBlocks();

      if (pixy.ccc.numBlocks) {
        piston2();
        Serial.println("Detected");
        pixy.setLamp(0, 0);
        break;
      }
    }
    // function();
    delay(200);
    servomotion(90,15);
    stoprollers();
    // delay(500);
    // dribble_arm_in();
    // digitalWrite(dribbling_arm_out,HIGH);
    // digitalWrite(dribbling_arm_in,LOW);
    // delay(500);
    // digitalWrite(dribbling_arm_out,LOW);
    // digitalWrite(dribbling_arm_in,LOW);
    // servo_stop.write(0);


    // feed_pid_timer.begin(feed_pid,10000);
    // rcv_pid_timer.begin(rcv_pid,10000);
    // feeding
    // sp1 = 3000;
  
  }
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
    servo_left.write(angle-3);
    //Serial.println(angle);
    delay(15);
    }
  }
}

void rollers(){
digitalWrite(roller1_pin,HIGH);
// analogWrite(roller1_spd,255*64);

}

void stoprollers(){
digitalWrite(roller1_pin,LOW);
// analogWrite(roller1_spd,0);

}

void piston1(){
digitalWrite(ball_push,LOW);
digitalWrite(ball_pull,HIGH);
delay(500);

digitalWrite(ball_push,HIGH);
digitalWrite(ball_pull,LOW);

delay(500);
digitalWrite(ball_push,LOW);
digitalWrite(ball_pull,LOW);  

}


void piston2(){
digitalWrite(extend,LOW);
digitalWrite(retract,HIGH);
delay(600);

digitalWrite(extend,HIGH);
digitalWrite(retract,LOW);
delay(500);

digitalWrite(extend,LOW);
digitalWrite(retract,LOW);

}

void dribble_arm_out(){
  for (int i = 0; i < 2; i++){
    limitSwitchState[i] = digitalRead(limitSwitch[i]);
  }
  while (limitSwitchState[1] != LOW){
    for (int i = 0; i < 2; i++){
      digitalWrite(dribble_arm_dir[i], HIGH);
      analogWrite(dribble_arm_pwm[i], 255*32);
    }

    limitSwitchState[1] = digitalRead(limitSwitch[1]);
    // limitSwitchState[3] = digitalRead(limitSwitch[3]);
  }

  for (int i = 0; i < 2; i++){
    digitalWrite(dribble_arm_dir[i], HIGH);
    analogWrite(dribble_arm_pwm[i], 255*0);
  }

}

void dribble_arm_in(){
  for (int i = 0; i < 2; i++){
    limitSwitchState[i] = digitalRead(limitSwitch[i]);
  }
  while (limitSwitchState[0] != LOW){
    for (int i = 0; i < 2; i++){
      digitalWrite(dribble_arm_dir[i], LOW);
      analogWrite(dribble_arm_pwm[i], 255*32);
    }

    limitSwitchState[0] = digitalRead(limitSwitch[0]);
    // limitSwitchState[2] = digitalRead(limitSwitch[2]);
  }

  for (int i = 0; i < 2; i++){
    digitalWrite(dribble_arm_dir[i], HIGH);
    analogWrite(dribble_arm_pwm[i], 255*0);
  }

}
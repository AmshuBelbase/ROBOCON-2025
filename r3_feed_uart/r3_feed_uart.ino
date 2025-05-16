#include "USBHost_t36.h"

int motor_dir_pin=5;
int motor_pwm_pin=4;
int limitswitch_up=19;
int limitswitch_down=17;
int counter=0;
bool switchup=1;
bool switchdown=0;

int rpm = 2000;
int stop_bldc = 0;

USBHost myusb;
USBHub hub1(myusb);
USBHIDParser hid1(myusb);
JoystickController joystick1(myusb);
// BluetoothController bluet (myusb, true, "0000");   // Version does pairing to device
BluetoothController bluet(myusb);  // version assumes it already was paireduint32_t buttons_prev = 0;
uint32_t buttons;

int psAxis[64];
int psAxis_prev[64];
bool first_joystick_message = true;

void ball_feed(){
  myusb.Task();
   if (joystick1.available()) {
    for (uint8_t i = 0; i < 64; i++) {
      psAxis_prev[i] = psAxis[i];
      psAxis[i] = joystick1.getAxis(i);
    }
     buttons = joystick1.getButtons();
    //  Serial.println(buttons);
    }
  switchup = digitalRead(limitswitch_up);
  switchdown = digitalRead(limitswitch_down);
  Serial.print("up:");
  Serial.print(switchup);
  Serial.print("down:");
  Serial.println(switchdown);
 
  Serial.println(counter); 

  // if(buttons==4 && (switchdown==0 || switchdown==1)){
  //   if(switchup==1){
  //     digitalWrite(motor_dir_pin,HIGH);
  //     analogWrite(motor_pwm_pin,64*255);
  //   }
  //   else if(switchup==0){
  //     digitalWrite(motor_dir_pin,LOW);
  //     analogWrite(motor_pwm_pin,0);

  //   }
  // }
  // else if(buttons==4 && switchup==0){
  //   if(switchdown==1){
  //   digitalWrite(motor_dir_pin,LOW);
  //   analogWrite(motor_pwm_pin,64*255);
  //   }
  //   else if(switchdowm==0){
  //     digitalWrite(motor_dir_pin,LOW);
  //     analogWrite(motor_pwm_pin,0);

  //   }
  // }
 


  if(counter==0){
    if(buttons==4){
      digitalWrite(motor_dir_pin,HIGH);
      analogWrite(motor_pwm_pin,64*255);
      // Serial1.write(rpm);
      Serial1.write((uint8_t*)&rpm, sizeof(rpm));
      // Serial.println("SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSStart");

    }
    if(switchup==0){
      digitalWrite(motor_dir_pin,LOW);
      analogWrite(motor_pwm_pin,0);
      // Serial.println("SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSStop");
      counter=1;
      delay(5000);

    }
  }
  else if(counter==1){
    if(buttons==4){
      digitalWrite(motor_dir_pin,LOW);
      analogWrite(motor_pwm_pin,64*255);
      Serial1.write(0);
      // Serial1.write((uint8_t*)&stop_bldc, sizeof(stop_bldc));
    }
    if(switchdown==0){
      digitalWrite(motor_dir_pin,HIGH);
      analogWrite(motor_pwm_pin,0);
      // Serial.println("SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS");
      counter=0;
    }
  }
 
}

void setup(){

  Serial.begin(9600);
  Serial1.begin(115200);
    
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  if (CrashReport) Serial.print(CrashReport);
  myusb.begin();

  analogWriteResolution(14);
  analogWriteFrequency(motor_pwm_pin,9000);

  pinMode(limitswitch_up,INPUT_PULLUP);
  pinMode(limitswitch_down,INPUT_PULLUP);
  pinMode(motor_dir_pin,OUTPUT);
  pinMode(motor_pwm_pin,OUTPUT);

  digitalWrite(motor_dir_pin,LOW);
  analogWrite(motor_pwm_pin,0);


}

void loop(){
  ball_feed(); 
}
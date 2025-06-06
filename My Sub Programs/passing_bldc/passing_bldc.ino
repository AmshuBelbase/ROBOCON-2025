#include <Pixy2I2C.h>
Pixy2I2C pixy;
#include <Servo.h>

int piston[4]={31,32,26,27};
int roller= 28;
int buttons = 0;


Servo servo_left;
Servo servo_right;

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
Serial8.begin(115200);

servo_left.attach(2);
  servo_right.attach(1);
  servo_left.write(180);
  servo_right.write(0);

    pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);

  pinMode(roller, OUTPUT);
  digitalWrite(roller,HIGH);

// analogWrite(13,255);
for(int i=0;i<4;i++)
  pinMode(piston[i],OUTPUT); 
pixy.init();
digitalWrite(piston[2],LOW);
digitalWrite(piston[3],HIGH);
digitalWrite(piston[0],LOW);
digitalWrite(piston[1],HIGH);

delay(500);
digitalWrite(piston[2],LOW);
digitalWrite(piston[3],LOW);
digitalWrite(piston[1],LOW);
digitalWrite(piston[0],LOW);


}

void servomotion(int start_angle, int end_angle){
  if(start_angle<end_angle){ 
    for(int angle=start_angle;angle<=end_angle;angle++){
    // if(angle<125)
    servo_right.write(angle);

    servo_left.write(180-angle);
    delay(15);
    }
  }
  else if(start_angle>end_angle){
    for(int angle=start_angle;angle>=end_angle;angle--){
    // if(angle<125)
    servo_right.write(angle);
    //Serial.println(180-angle);
    servo_left.write(180-angle);
    //Serial.println(angle);
    delay(15);
    }
  }     
}


String input="";
void loop() {
  // put your main code here, to run repeatedly:
if(Serial.available())
{
  input=Serial.readStringUntil('\n');

  
}

if (Serial8.available() >= sizeof(buttons)){
  Serial8.readBytes((char*)&buttons, sizeof(buttons));
  Serial.print("button: ");
  Serial.println(buttons);
}

// Serial.println(input);
// Serial.println("100");
if(buttons == 4 || input=="s")
{

  servomotion(0,99);
  delay(100);
  digitalWrite(piston[2],HIGH);
  digitalWrite(piston[3],LOW);
  delay(500);
  digitalWrite(piston[2],LOW);
  digitalWrite(piston[3],LOW);
  delay(500);
  
  digitalWrite(piston[2],LOW);
  digitalWrite(piston[3],HIGH);
  delay(400);

  digitalWrite(roller,LOW);
  digitalWrite(piston[2],LOW);
  digitalWrite(piston[3],LOW);
  delay(400);
  pixy.setLamp(1, 1);
    while (true) {
      Serial.println("pixy_while");
      pixy.ccc.getBlocks();
      if (pixy.ccc.numBlocks) {
        digitalWrite(piston[1],LOW);
        digitalWrite(piston[0],HIGH); 
        delay(500);
        digitalWrite(piston[1],HIGH);
        digitalWrite(piston[0],LOW);
        delay(500);
        Serial.println("Detected");
        pixy.setLamp(0, 0);
        break;
      }
    }
      digitalWrite(roller,HIGH);
      servomotion(99,0);


        digitalWrite(piston[0],LOW);
        digitalWrite(piston[1],LOW);
        digitalWrite(piston[2],LOW);
        digitalWrite(piston[3],LOW);

            input="";

}

}





// #include "USBHost_t36.h"

// int motor_dir_pin=5;
// int motor_pwm_pin=4;
// int limitswitch_up=16;
// int limitswitch_down=17;
// int counter=0;
// bool switchup=1;
// bool switchdown=0;
 
// USBHost myusb;
// USBHub hub1(myusb);
// USBHIDParser hid1(myusb);
// JoystickController joystick1(myusb);
// // BluetoothController bluet (myusb, true, "0000");   // Version does pairing to device
// BluetoothController bluet(myusb);  // version assumes it already was paireduint32_t buttons_prev = 0;
// uint32_t buttons;

// int psAxis[64];
// int psAxis_prev[64];
// bool first_joystick_message = true;

// void ball_feed(){
//   myusb.Task();
//    if (joystick1.available()) {
//     for (uint8_t i = 0; i < 64; i++) {
//       psAxis_prev[i] = psAxis[i];
//       psAxis[i] = joystick1.getAxis(i);
//     }
//      buttons = joystick1.getButtons();
//     //  Serial.println(buttons);
//     }
//   switchup = digitalRead(limitswitch_up);
//   switchdown = digitalRead(limitswitch_down);
//   Serial.print("up:");
//   Serial.print(switchup);
//   Serial.print("down:");
//   Serial.println(switchdown);
 
//   // Serial.println(counter); 
 
//   if(buttons==4 && switchdown==0){
//     Serial.print("Going Up");
//     Serial.print(switchup);
//     int cnt = 0;
//     while(cnt<=5){
//       digitalWrite(motor_dir_pin,HIGH);
//       analogWrite(motor_pwm_pin,64*255);
//       switchup = digitalRead(limitswitch_up);  
//       if(switchup == 0){
//         cnt++;
//       }else{
//         cnt = 0;
//       }
//     }
//     digitalWrite(motor_dir_pin,LOW);
//     analogWrite(motor_pwm_pin,0);
//     Serial.print("Reached Up");
//     Serial.print(switchup); 
//   }
//   else if(buttons==4){
//     Serial.print("Going Down");
//     while(switchdown!=0){
//       digitalWrite(motor_dir_pin,LOW);
//       analogWrite(motor_pwm_pin,64*255); 
//       switchdown = digitalRead(limitswitch_down);
//       Serial.print(".");
//     }
//     digitalWrite(motor_dir_pin,HIGH);
//     analogWrite(motor_pwm_pin,0);
//     Serial.print("Reached Down"); 
//   }
 


//   // if(counter==0){
//   //   if(buttons==4){
//   //     digitalWrite(motor_dir_pin,HIGH);
//   //     analogWrite(motor_pwm_pin,64*255);
//   //     // Serial.println("SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSStart");

//   //   }
//   //   if(switchup==0){
//   //     digitalWrite(motor_dir_pin,LOW);
//   //     analogWrite(motor_pwm_pin,0);
//   //     // Serial.println("SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSStop");
//   //     counter=1;
//   //     delay(5000);

//   //   }
//   // }
//   // else if(counter==1){
//   //   if(buttons==4){
//   //     digitalWrite(motor_dir_pin,LOW);
//   //     analogWrite(motor_pwm_pin,64*255);
//   //   }
//   //   if(switchdown==0){
//   //     digitalWrite(motor_dir_pin,HIGH);
//   //     analogWrite(motor_pwm_pin,0);
//   //     // Serial.println("SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS");
//   //     counter=0;
//   //   }
//   // }
 
// }

// void setup(){

//   Serial.begin(9600);
    
//   pinMode(13, OUTPUT);
//   digitalWrite(13, HIGH);

//   if (CrashReport) Serial.print(CrashReport);
//   myusb.begin();

//   analogWriteResolution(14);
//   analogWriteFrequency(motor_pwm_pin,9000);

//   pinMode(limitswitch_up,INPUT_PULLUP);
//   pinMode(limitswitch_down,INPUT_PULLUP);
//   pinMode(motor_dir_pin,OUTPUT);
//   pinMode(motor_pwm_pin,OUTPUT);

//   digitalWrite(motor_dir_pin,LOW);
//   analogWrite(motor_pwm_pin,0);


// }

// void loop(){
//   ball_feed(); 
// }

// // /*
// //   Name:    setCurrent.ino
// //   Created: 19-08-2018
// //   Author:  SolidGeek
// //   Description: This is a very simple example of how to set the current for the motor
// // */

// // #include <VescUart.h>
// // #include <Encoder.h>

// // /** Initiate VescUart class */
// // VescUart UART;

// // IntervalTimer feed_pid_timer;
// // float current = 0; /** The current in amps */

// // Encoder encFeed(7,6);
// // // int feed = 0;

// // int feeder_pwm=18;
// // int feeder_dir=16;

// // int feeder_cpr=538;
// // float ap_count_feeder=0;
// // float sp_angle_feeder=0;
// // float sp_count_feeder=0;

// // float kP=150,kI=5,kD=1;

// // float err_feed=0,der_feed=0,integ_feed=0,lastError_feed=0;
// // float speed_feed=0;


// // void setup() {
// //   Serial.begin(2000000);
// //   /** Setup UART port (Serial1 on Atmega32u4) */
// //   Serial1.begin(115200);
  
// //   while (!Serial8) {;}
// //   analogWrite(13,255);
// //   /** Define which ports to use as UART */
// //   UART.setSerialPort(&Serial1);

// //   // feed_pid_timer.begin(feed_pos_pid,10000);

// //   pinMode(feeder_pwm,OUTPUT);
// //   pinMode(feeder_dir,OUTPUT);

// //   analogWriteResolution(14);
// //   analogWriteFrequency(0, 9000);
// // }
// //   int nrpm=0;
// //   int orpm = 0;

// // void setPosition(int pwm,int dir ,int speed)
// // {
// //   digitalWrite(dir,speed<0?LOW:HIGH);
// //   analogWrite(pwm,abs(speed));
// // }

// // void feed_pos_pid()
// // {

// // ap_count_feeder=encFeed.read();
// // sp_count_feeder=sp_angle_feeder*feeder_cpr/360.0;

// // err_feed=sp_count_feeder-ap_count_feeder;
// // der_feed=(err_feed-lastError_feed)/0.01;
// // integ_feed=integ_feed+(err_feed-lastError_feed)*0.01;
// // lastError_feed=err_feed;

// // speed_feed=kP*err_feed+kI*integ_feed+kD*der_feed;
// // constrain(speed_feed,-14361,14361);
// // // Serial.printf("speed: %f   ",speed_m);
// // // Serial.printf("Kp: %f.   Ki: %f.    Kd: %f   ",kp,ki,kd);
// // // Serial.print("Sp_feed:");
// // // Serial.print(sp_angle_feed);
// // // Serial.print("     Ap_feed:");
// // // Serial.println(int((ap_angle_feed*360.0/shooter_rotation_cpr)));
// // // //       // Serial.println(enc1.read());
// // setPosition(feeder_pwm,feeder_dir,int(speed_feed));

// // }


// // void loop() {
  
// //   /** Call the function setCurrent() to set the motor current */

// //     if (Serial.available() > 0) {
// //                  String input = Serial.readString();//15001
// //                 // Serial.println(input.length());
      
// //                   nrpm=input.toFloat()*7;
// //                   // feed=input.substring(5).toInt();
                 
// //                }
// //     if(nrpm != 0){
// //       // sp_angle_feeder = -3300;
// //       while(orpm < nrpm){
// //         orpm=orpm+((nrpm-orpm)/700)+((nrpm-orpm)%700);
// //         UART.setRPM(orpm);
// //         delay(50);
// //         Serial.println("1");
// //         Serial.println((orpm/7));
// //       }
// //       while(orpm > nrpm){
// //         orpm=orpm-((orpm-nrpm)/700)-((orpm-nrpm)%700);
// //         UART.setRPM(orpm);
// //         delay(50);
// //         Serial.println("2");
// //         Serial.println((orpm/7));
// //       } 
// //     }else{
// //       orpm = 0;
// //       // sp_angle_feeder = 0;

// //     }
// //     UART.setRPM(orpm);
// //     if ( UART.getVescValues() ) {
// //       if(UART.data.avgInputCurrent > 15){
// //         Serial.println("Values: ");
// //         // Serial.println(UART.data.rpm/7);
// //         // Serial.println((UART.data.inpVoltage)/10);
// //         // Serial.println(UART.data.ampHours);
// //         // Serial.println(UART.data.pidPos);
// //         // Serial.println(UART.data.avgMotorCurrent);
// //         Serial.println(UART.data.avgInputCurrent);
// //         // Serial.println(UART.data.tachometer);
// //     }
// //   }

// // //                Serial.print()
// // // Serial.print(current);
// //   //UART.setCurrent(current);
// //   // if (feed == 1){
// //   //   UART.setRPM(orpm);
// //   //   sp_angle_feeder=3550;
// //   // }else if (feed == 0){
// //   //   UART.setRPM(0);
// //   //   sp_angle_feeder = 0;

// //   // }


    

// //   // if ( UART.getVescValues() ) {
// //   //   Serial.println("Values: ");
// //   //   Serial.println(UART.data.rpm/7);
// //   //   Serial.println(UART.data.inpVoltage);
// //   //   Serial.println(UART.data.ampHours);
// //   //   Serial.println(UART.data.pidPos);

// //   //   Serial.println(UART.data.avgMotorCurrent);
// //   //   Serial.println(UART.data.avgInputCurrent);

// //   //   Serial.println(UART.data.tachometer);

// //   // }

// //   // UART.setBrakeCurrent(current);
  
// // }
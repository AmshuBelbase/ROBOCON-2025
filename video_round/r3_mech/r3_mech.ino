// #include <Pixy2I2C.h>
// Pixy2I2C pixy;
#include <Servo.h>

int piston[4]={31,32,26,27};
int roller= 28;
int buttons = 0;


Servo servo_left;
Servo servo_right;
Servo servo_stop;


int motor_dir_pin=5;
int motor_pwm_pin=4;
int limitswitch_up=16;
int limitswitch_down=17;
int counter=0;
bool switchup=1;
bool switchdown=0;



void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
Serial8.begin(115200);

servo_left.attach(2);
  servo_right.attach(1);
  servo_left.write(180);
  servo_right.write(0);
  servo_stop.attach(0);
  // for(int i=0i<100;i++)
      pinMode(13,OUTPUT);
  digitalWrite(13,LOW);

// pixy.init();

  digitalWrite(13,HIGH);

  pinMode(roller, OUTPUT);
  digitalWrite(roller,HIGH);

// analogWrite(13,255);
for(int i=0;i<4;i++)
  pinMode(piston[i],OUTPUT); 
digitalWrite(piston[2],LOW);
digitalWrite(piston[3],HIGH);
digitalWrite(piston[0],LOW);
digitalWrite(piston[1],HIGH);

delay(500);
digitalWrite(piston[2],LOW);
digitalWrite(piston[3],LOW);
digitalWrite(piston[1],LOW);
digitalWrite(piston[0],LOW);


analogWriteResolution(14);
analogWriteFrequency(motor_pwm_pin,9000);

pinMode(limitswitch_up,INPUT_PULLUP);
pinMode(limitswitch_down,INPUT_PULLUP);
pinMode(motor_dir_pin,OUTPUT);
pinMode(motor_pwm_pin,OUTPUT);

digitalWrite(motor_dir_pin,LOW);
analogWrite(motor_pwm_pin,0);


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


// String input="";
void loop() {
  // put your main code here, to run repeatedly:
// if(Serial.available())
// {
//   input=Serial.readStringUntil('\n');

  
// }

if (Serial8.available() >= sizeof(buttons)){
  Serial8.readBytes((char*)&buttons, sizeof(buttons));
  Serial.print("button: ");
  Serial.println(buttons);
}
if(buttons==8){
    digitalWrite(motor_dir_pin,HIGH);
    analogWrite(motor_pwm_pin,55*255);
  }else if(buttons==2){
    digitalWrite(motor_dir_pin,LOW);
    analogWrite(motor_pwm_pin,55*255);
  }else if(buttons==1){
    digitalWrite(motor_dir_pin,HIGH);
    analogWrite(motor_pwm_pin,0*255);
  }
  else if(buttons==4)
  {
    for(int i=55;i>=0;i--)
        {servo_stop.write(i);
        delay(20);}
  }

// Serial.println(input);
// Serial.println("100");
// if(buttons == 4 || input=="s")
// {

//   servomotion(0,95);
//   delay(100);
//   digitalWrite(piston[2],HIGH);
//   digitalWrite(piston[3],LOW);
//   delay(500);
//   digitalWrite(piston[2],LOW);
//   digitalWrite(piston[3],LOW);
//   delay(500);
  
//   digitalWrite(piston[2],LOW);
//   digitalWrite(piston[3],HIGH);
//   delay(400);

//   digitalWrite(roller,LOW);
//   digitalWrite(piston[2],LOW);
//   digitalWrite(piston[3],LOW);
//   delay(400);
//   pixy.setLamp(1, 1);
//     while (true) {
//       Serial.println("pixy_while");
//       pixy.ccc.getBlocks();
//       if (pixy.ccc.numBlocks) {
//         digitalWrite(piston[1],LOW);
//         digitalWrite(piston[0],HIGH); 
//         delay(500);
//         digitalWrite(piston[1],HIGH);
//         digitalWrite(piston[0],LOW);
//         delay(500);
//         Serial.println("Detected");
//         pixy.setLamp(0, 0);
//         break;
//       }
//     }
//       digitalWrite(roller,HIGH);
//       servomotion(95,0);


//         digitalWrite(piston[0],LOW);
//         digitalWrite(piston[1],LOW);
//         digitalWrite(piston[2],LOW);
//         digitalWrite(piston[3],LOW);

//             input="";

// }


//   switchup = digitalRead(limitswitch_up);
//   switchdown = digitalRead(limitswitch_down);
// Serial.println(switchup);
// Serial.println(switchdown);

// if(counter==0){
//     if(buttons==2){
//       digitalWrite(motor_dir_pin,HIGH);
//       analogWrite(motor_pwm_pin,64*255);
//       // Serial.println("SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSStart");

//     }
//     if(switchup==0){
//       digitalWrite(motor_dir_pin,LOW);
//       analogWrite(motor_pwm_pin,0);
//       // Serial.println("SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSStop");
//       counter=1;

//     }
//   }
//   else if(counter==1){
//     if(buttons==2){
//       digitalWrite(motor_dir_pin,LOW);
//       analogWrite(motor_pwm_pin,64*255);
//     }
//     if(switchdown==0){
//       digitalWrite(motor_dir_pin,HIGH);
//       analogWrite(motor_pwm_pin,0);
//       // Serial.println("SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS");
//       counter=0;
//     }
//   }
}
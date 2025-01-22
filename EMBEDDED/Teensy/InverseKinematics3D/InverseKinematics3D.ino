#include <USBHost_t36.h>

USBHost mysub;
JoystickController ps4(mysub);
BluetoothController bt(mysub , true , "0000");
USBHIDParser hid1(mysub);

int dir1=2;
int speed1=3;

int dir2=6;
int speed2=7;

int dir3=4;
int speed3=5;

float v1_=0;
float v2_=0;
float v3_=0;

void setup(){
  Serial.begin(9600);
  pinMode(dir1,OUTPUT);
  pinMode(dir2,OUTPUT);
  pinMode(dir3,OUTPUT);
  pinMode(speed1,OUTPUT);
  pinMode(speed2,OUTPUT);
  pinMode(speed3,OUTPUT);
  mysub.begin();
  delay(5000);
 
  Serial.println("Connecting to PS4 controller");
}


void loop(){
  mysub.Task();

  if(ps4.available()){
    int LX = ps4.getAxis(0);
    int LY = ps4.getAxis(1);
    int RY = ps4.getAxis(5);
    LX=map(LX,0,255,-127,127);
    LY=map(LY,0,255,127,-127);
    RY=map(RY,0,255,127,-127);

    Serial.print("LX: ");
    Serial.println(LX);

    Serial.print("LY: ");
    Serial.println(LY);

    Serial.print("RY: ");
    Serial.println(RY);

    Serial.print("Sum of LX LY: ");
    Serial.println(abs(LX)+abs(LY));

    Serial.print("Sum of LX LY RY: ");
    Serial.println(abs(LX)+abs(LY)+abs(RY));

    v1_ = -(2.0/3.0)*LX ;
    v2_ = (1.0/3.0)*LX - (1.0/1.732)*LY ;
    v3_ = (1.0/3.0)*LX + (1.0/1.732)*LY ;
    float v1 = map(v1_,-84.67,84.67,-255.0,255.0);
    float v2 = map(v2_,-85.0,85.0,-255.0,255.0);
    float v3 = map(v3_,-85.0,85.0,-255.0,255.0);
    float vw = map(RY,-127,127,-255,255);
    if(RY==0){
    if(v1<0){
      digitalWrite(dir1,LOW);
      analogWrite(speed1,abs(v1));
    }
    else{
      digitalWrite(dir1,HIGH);
      analogWrite(speed1,abs(v1));
    }

    if(v2<0){
      digitalWrite(dir2,LOW);
      analogWrite(speed2,abs(v2));
    }
    else{
      digitalWrite(dir2,HIGH);
      analogWrite(speed2,abs(v2));
    }

    if(v3<0){
      digitalWrite(dir3,LOW);
      analogWrite(speed3,abs(v3));
    }
    else{
      digitalWrite(dir3,HIGH);
      analogWrite(speed3,abs(v3));
    }
    }
    else{
      if(vw<0){
        digitalWrite(dir1,LOW);
        digitalWrite(dir2,LOW);
        digitalWrite(dir3,LOW);

        analogWrite(speed1,abs(vw));
        analogWrite(speed2,abs(vw));
        analogWrite(speed3,abs(vw));
      }
      else{
        digitalWrite(dir1,HIGH);
        digitalWrite(dir2,HIGH);
        digitalWrite(dir3,HIGH);

        analogWrite(speed1,abs(vw));
        analogWrite(speed2,abs(vw));
        analogWrite(speed3,abs(vw));
      }
    }
    
    Serial.print("V1_: ");
    Serial.println(v1_);

    Serial.print("V2_: ");
    Serial.println(v2_);

    Serial.print("V3_: ");
    Serial.println(v3_);




    Serial.print("V1: ");
    Serial.println(v1);

    Serial.print("V2: ");
    Serial.println(v2);

    Serial.print("V3: ");
    Serial.println(v3);

    Serial.print("VW: ");
    Serial.println(vw);

    if(LX==0&&RY==0&&LY==0){
      analogWrite(speed1,0);
      analogWrite(speed2,0);
      analogWrite(speed3,0);
    }
  }
  else
    Serial.println("Not connected");
  delay(100);
}
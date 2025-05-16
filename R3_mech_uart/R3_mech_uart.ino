#include <Pixy2I2C.h>
Pixy2I2C pixy;

// feeding
int motor_dir_pin=5;
int motor_pwm_pin=4;
int limitswitch_up=19;
int limitswitch_down=17;
int counter=0;
bool switchup=1;
bool switchdown=0;

// dribbling
int piston[4]={31,32,26,27};

int buttons = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial8.begin(115200);    //teensy uart

  analogWriteResolution(14);
  analogWriteFrequency(0, 9000);

  analogWrite(13,255);
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

  pinMode(limitswitch_up,INPUT_PULLUP);
  pinMode(limitswitch_down,INPUT_PULLUP);
  pinMode(motor_dir_pin,OUTPUT);
  pinMode(motor_pwm_pin,OUTPUT);

  digitalWrite(motor_dir_pin,LOW);
  analogWrite(motor_pwm_pin,0);


}
// String input="";
void loop() {


  if (Serial8.available() >= sizeof(buttons)){
    Serial8.readBytes((char*)&buttons, sizeof(buttons));
    Serial.print("button: ");
    Serial.println(buttons);
  }

  ball_feed();

  if(buttons == 4)
  {
    digitalWrite(piston[2],HIGH);
    digitalWrite(piston[3],LOW);
    delay(500);
    digitalWrite(piston[2],LOW);
    digitalWrite(piston[3],LOW);
    delay(500);
    
    digitalWrite(piston[2],LOW);
    digitalWrite(piston[3],HIGH);
    delay(400);
    digitalWrite(piston[2],LOW);
    digitalWrite(piston[3],LOW);
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

    digitalWrite(piston[0],LOW);
    digitalWrite(piston[1],LOW);
    digitalWrite(piston[2],LOW);
    digitalWrite(piston[3],LOW);

  }
}

void ball_feed(){
  switchup = digitalRead(limitswitch_up);
  switchdown = digitalRead(limitswitch_down);
  // Serial.print("up:");
  // Serial.print(switchup);
  // Serial.print("down:");
  // Serial.println(switchdown);
 
  Serial.println(counter); 
  if(counter==0){
    if(buttons==2){
      digitalWrite(motor_dir_pin,HIGH);
      analogWrite(motor_pwm_pin,32*255);
      // Serial.println("SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSStart");

    }
    if(switchup==0){
      digitalWrite(motor_dir_pin,LOW);
      analogWrite(motor_pwm_pin,0);
      // Serial.println("SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSStop");
      counter=1;

    }
  }
  else if(counter==1){
    if(buttons==2){
      digitalWrite(motor_dir_pin,LOW);
      analogWrite(motor_pwm_pin,32*255);
    }
    if(switchdown==0){
      digitalWrite(motor_dir_pin,HIGH);
      analogWrite(motor_pwm_pin,0);
      // Serial.println("SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS");
      counter=0;
    }
  }
 
}
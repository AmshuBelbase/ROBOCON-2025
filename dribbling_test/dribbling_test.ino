#include<Servo.h>
#include <Wire.h>
IntervalTimer myTimer;

int roller_dir=12;
int roller_pwm=11;
bool flag_roller = 0;

Servo servo_left;
Servo servo_right;
Servo servo_stopper;
int piston[3][2] = { {26,27} , {31,30} , {28,29} };

void setup(){
  Serial.begin(9600);
  analogWriteResolution(14);
  analogWriteFrequency(0, 9000);

  servo_left.attach(2);
  servo_right.attach(1);
  servo_stopper.attach(3);

  servo_left.write(15);
  servo_right.write(180-15);
  servo_right.write(180-98-6);
  servo_left.write(98-3);
  servo_stopper.write(0);

  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);

  pinMode(roller_dir, OUTPUT);
  pinMode(roller_pwm, OUTPUT);
  // pinMode(roller1_spd, OUTPUT);
  for (int i = 0; i < 3; i++){
    for (int j = 0; j < 2; j++){
      pinMode(piston[i][j], OUTPUT);
    }
  }

  pistonControl(0, false);
  pistonControl(1, false);
  pistonControl(2, false);

}

void servomotion(int start_angle, int end_angle){
  if(start_angle<end_angle){ 
    for(int angle=start_angle;angle<=end_angle;angle++){
      servo_right.write(180-angle);
      servo_left.write(angle-3);
      delay(15);
    }     
  }
  else if(start_angle>end_angle){
    for(int angle=start_angle;angle>=end_angle;angle--){
      servo_right.write(180-angle);
      servo_left.write(angle-3);
      delay(15);
    }
  }
}

void rollers(){
  digitalWrite(roller_dir, !flag_roller);
  analogWrite(roller_pwm, flag_roller?0:(255*64));
  flag_roller = !flag_roller;
}

void pistonControl(int pistonNum, bool extend){
  digitalWrite(piston[pistonNum][0], extend);
  digitalWrite(piston[pistonNum][1], !extend);
  delay(1000);
  digitalWrite(piston[pistonNum][0], LOW);
  digitalWrite(piston[pistonNum][1], LOW);

}

int value=0;
void loop()
{
  // servomotion(15,96);
  //Serial.println("Okay");
  if (Serial.available() > 0) {  // Check if data is available
        String input = Serial.readStringUntil('\n');  // Read input
        input.trim();  // Remove any whitespace
        value = input.toInt();  // Convert the entire string to an integer

        //Serial.print("Received value: ");
  }              
  Serial.println(value);


  if(value==1){
    servo_stopper.write(90);
    pistonControl(1, true);
    // servomotion(15,96);
    delay(500);

    pistonControl(0, true);
    delay(100);
    pistonControl(0, false);

    delay(50); //200
    rollers(); 
    // delay(2000);
    value=-1;
  }
  else if(value==2){
    servo_stopper.write(0);
    // servomotion(96,15);
    rollers();
    value=-2;
  }
  else if(value==3){
    pistonControl(2, true);
    delay(1000);
    pistonControl(2, false);
    value = -3;
  }
  else if(value==4){
    pistonControl(1, true);
    delay(3000);
    pistonControl(1, false);
    value = -4;
  }
 
}
/*Filename: r1_mech.ino
  Date: April 12, 202

  -----------------------------------
   DRIBBLING SYSTEM
  -----------------------------------
  Pin 2   -> Servo - Left Arm (servo_left)
  Pin 1   -> Servo - Right Arm (servo_right)
  Pin 3   -> Servo - Stop Servo (servo_stop)
  Pin 11  -> Roller 1 Enable (Digital HIGH/LOW)
  Pin 12  -> Roller 1 Speed (PWM - analogWrite)
  Pin 30  -> Dribbling Arm Out (Piston OUT)
  Pin 31  -> Dribbling Arm In (Piston IN)
  Pin 29  -> Piston 1 (Ball Control)
  Pin 28  -> Piston 2 (Ball Control)
  Pin 27  -> Ball Push (Pulse HIGH)
  Pin 26  -> Ball Pull (Pulse HIGH)

  -----------------------------------
   FEEDING SYSTEM
  -----------------------------------
  Pin 19  -> Feeder PWM (Motor Speed)
  Pin 16  -> Feeder Direction (Motor Dir)

  -----------------------------------
   RECEIVING SYSTEM
  -----------------------------------
  Pin 23  -> RCV Right PWM (Motor Speed)
  Pin 22  -> RCV Left PWM (Motor Speed)
  Pin 21  -> RCV Right DIR
  Pin 20  -> RCV Left DIR

  -----------------------------------
   LIMIT SWITCHES (Using pullups)
  -----------------------------------
  Pin 14  -> limitSwitch[0] -> Feeder Bottom Limit
  Pin 15  -> limitSwitch[1] -> Feeder Top Limit
  Pin 41  -> limitSwitch[2] -> Right Down Limit
  Pin 40  -> limitSwitch[3] -> Right Up Limit
  Pin 38  -> limitSwitch[4] -> Left Down Limit
  Pin 39  -> limitSwitch[5] -> Left Up Limit

  -----------------------------------
   COMMUNICATION PORTS
  -----------------------------------
  Serial    -> USB Debug Monitor (200000 baud)
  Serial2   -> Button Input (115200 baud)
  Serial7   -> VESC UART for BLDC motor (115200 baud)

  ============================
*/


#include <Encoder.h>
#include <VescUart.h>
#include <Servo.h>
#include <Pixy2I2C.h>
// Pixy2I2C pixy;

VescUart UART;

IntervalTimer feed_pid_timer;
IntervalTimer rcv_pid_timer;

int flag_rec_r=0;
int flag_rec_l=0;
int flag_feed=0;

bool limitSwitchState[8] = {false};
int limitSwitch[6]={14,15,41,40,38,39};

//driblling 
int roller1_pin=11;//left
int roller1_spd=12;

int dribbling_arm_out=30;
int dribbling_arm_in=31;

int piston1=29;
int piston2=28;

int ball_push=27;
int ball_pull=26;

Servo servo_left;
Servo servo_right;
Servo servo_stop;
/////////dribbling


// feeding
// Encoder myEnc_feed(15,14);   // feeding 
long currentCounts1 = 0;
volatile int feeder_pwm=19;
volatile int feeder_dir=16;

volatile float sp1;
float pid1 = 0.0;
float err1 = 0.0;
float prev_err1 = 0.0;
float integ1 = 0.0;
float der1 = 0.0; 


///////// feeding


// recieving 
// Encoder myEnc_rcv[2] = {Encoder(40,41), Encoder(39,38)};     //right, left

int rcv_pwm[2] = {23, 22};
int rcv_dir[2] = {21, 20};

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

  Serial.begin(200000);
  Serial2.begin(115200);
  Serial7.begin(115200);             // bldc
  UART.setSerialPort(&Serial7);

  // teensy led



  // dribbling
  // pixy.init();

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  servo_left.attach(2);
  servo_right.attach(1); 
  servo_stop.attach(3);
  servo_stop.write(0);
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


  pinMode(piston1,OUTPUT);
  pinMode(piston2,OUTPUT);

  digitalWrite(piston1,HIGH);
  digitalWrite(piston2,LOW);
  delay(1000);
  digitalWrite(piston1,LOW);
  digitalWrite(piston2,LOW);


  pinMode(dribbling_arm_out,OUTPUT);
  pinMode(dribbling_arm_in,OUTPUT);

  digitalWrite(dribbling_arm_out,HIGH);
  digitalWrite(dribbling_arm_in,LOW);
  delay(1000);
  digitalWrite(dribbling_arm_out,LOW);
  digitalWrite(dribbling_arm_in,LOW);


  // feeding
  pinMode(feeder_pwm, OUTPUT);
  pinMode(feeder_dir, OUTPUT);

  analogWrite(feeder_pwm, 0);
  digitalWrite(feeder_dir, LOW);

  // pinMode(s1, INPUT_PULLUP);
  // pinMode(s2, INPUT_PULLUP);

for(int i=0;i<6;i++)
{
  pinMode(limitSwitch[i],INPUT_PULLUP);
}

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

  // feed_pid_timer.begin(feed_pid, 10000);
  // rcv_pid_timer.begin(rcv_pid, 10000);

}

int constraint(int value, int min_val, int max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}


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
digitalWrite(ball_push,LOW);
digitalWrite(ball_pull,LOW);  

}


void function(){
digitalWrite(piston1,LOW);
digitalWrite(piston2,HIGH);
delay(600);

digitalWrite(piston1,HIGH);
digitalWrite(piston2,LOW);
delay(500);

digitalWrite(piston1,LOW);
digitalWrite(piston2,LOW);

}

void dribble_arm(){
digitalWrite(piston1,HIGH);
digitalWrite(piston2,LOW);
delay(1000);

digitalWrite(piston1,LOW);
digitalWrite(piston2,HIGH);
delay(500);

digitalWrite(piston1,LOW);
digitalWrite(piston2,LOW);

}

/////DRIBBLING


int buttons = 0;
int lastTime=0;

void loop() {
  // Serial.println("ok");
    if (limitSwitchState[3] == LOW)
    {
      flag_rec_r = 1;

    }
    if (limitSwitchState[5] == LOW)
    {
      flag_rec_l = 1;
    }

        if (limitSwitchState[2] == LOW)
    {
      flag_rec_r = 0;

    }
    if (limitSwitchState[4] == LOW)
    {
      flag_rec_l = 0;
    }
    for (int i = 0; i < 6; i++){
      limitSwitchState[i] = digitalRead(limitSwitch[i]);

    }
    if(millis()-lastTime>=1000){
    for(int i =0;i<6;i++)
    {
      Serial.print(limitSwitchState[i]);
      
    }
    Serial.printf("    %d %d %d",flag_feed,flag_rec_r,flag_rec_l);
    Serial.println();
    lastTime=millis();
    }
  // Serial.println();
// delay(1000);
  if (Serial2.available() >=  sizeof(buttons)) {
    // Read buttons
    Serial2.readBytes((char*)&buttons, sizeof(buttons));
    Serial.print(" | Buttons: ");
    // buttons=buttons;
    Serial.println(buttons);
  }
  if (buttons == 2 && flag_feed == 0) {
    buttons = 0;
    // UART.setRPM(1500);
    // sp1=-3500;
    // flag_feed=1;
    Serial.println("Feed UP");
    while (limitSwitchState[1] != LOW){
      digitalWrite(feeder_dir, LOW);
      analogWrite(feeder_pwm, 255*64);
      limitSwitchState[1] = digitalRead(limitSwitch[1]);
    }
    digitalWrite(feeder_dir, LOW);
    analogWrite(feeder_pwm, 255*0);
    flag_feed = 1;

  }
  else if(buttons == 2 && flag_feed == 1){
    buttons = 0;
    Serial.println("Feed DOWN");
    // sp1=0;
    // flag_feed=0;
    // UART.setRPM(0);
    while (limitSwitchState[0] != LOW){
      digitalWrite(feeder_dir, HIGH);
      analogWrite(feeder_pwm, 255*64);
      limitSwitchState[0] = digitalRead(limitSwitch[0]);
    }
    digitalWrite(feeder_dir, LOW);
    analogWrite(feeder_pwm, 255*0);
    flag_feed = 0;
  }
  else if (buttons == 8 && flag_rec_r == 0 && flag_rec_l == 0) {
    buttons = 0;
    // Rsp[0]=5000;
    // Rsp[1]=5000;
    // flag_rec=1;
    Serial.println("REC UP");
    while (limitSwitchState[3] != LOW || limitSwitchState[5] != LOW){

      // for (int i = 0; i<2; i++){
        if(limitSwitchState[3] != LOW)
        {
          digitalWrite(rcv_dir[0], HIGH);
          analogWrite(rcv_pwm[0], 255*64);
        }
        else
        {
          digitalWrite(rcv_dir[0], HIGH);
          analogWrite(rcv_pwm[0], 255*0);
          flag_rec_r = 1;
        }
        if(limitSwitchState[5] != LOW)
        {
          digitalWrite(rcv_dir[1], HIGH);
          analogWrite(rcv_pwm[1], 255*64);
        }
        else
        {
          digitalWrite(rcv_dir[1], HIGH);
          analogWrite(rcv_pwm[1], 255*0);
          flag_rec_l = 1;
        }
      limitSwitchState[3] = digitalRead(limitSwitch[3]);
      limitSwitchState[5] = digitalRead(limitSwitch[5]);
       }
      limitSwitchState[3] = digitalRead(limitSwitch[3]);
      limitSwitchState[5] = digitalRead(limitSwitch[5]);
      

      digitalWrite(rcv_dir[0], HIGH);
          analogWrite(rcv_pwm[0], 255*0);
          digitalWrite(rcv_dir[1], HIGH);
          analogWrite(rcv_pwm[1], 255*0);
    
    // for (int i = 0; i<2; i++){
    //     digitalWrite(rcv_dir[i], HIGH);
    //     analogWrite(rcv_pwm[i], 255*0);

    //   }



  }
  else if(buttons == 8 && flag_rec_r == 1 && flag_rec_l == 1)
  {
    Serial.println("REC DOWN");
    buttons = 0;
    // Rsp[0]=0;
    // Rsp[1]=0;
    // flag_rec=0;

    // for (int i = 0; i<2; i++){
    //     digitalWrite(rcv_dir[i], LOW);
    //     analogWrite(rcv_pwm[i], 255*32);

    // }

    while (limitSwitchState[2] != LOW || limitSwitchState[4] != LOW){
      if(limitSwitchState[2] != LOW)
        {
          digitalWrite(rcv_dir[0], LOW);
          analogWrite(rcv_pwm[0], 255*64);
        }
        else
        {
          digitalWrite(rcv_dir[0], HIGH);
          analogWrite(rcv_pwm[0], 255*0);
          flag_rec_r=0;
        }
        if(limitSwitchState[4] != LOW)
        {
          digitalWrite(rcv_dir[1], LOW);
          analogWrite(rcv_pwm[1], 255*64);
        }
        else
        {
          digitalWrite(rcv_dir[1], HIGH);
          analogWrite(rcv_pwm[1], 255*0);
          flag_rec_l=0;
        }
        limitSwitchState[2] = digitalRead(limitSwitch[2]);
      limitSwitchState[4] = digitalRead(limitSwitch[4]);
        }

      limitSwitchState[2] = digitalRead(limitSwitch[2]);
      limitSwitchState[4] = digitalRead(limitSwitch[4]);
      
      digitalWrite(rcv_dir[1], HIGH);
          analogWrite(rcv_pwm[1], 255*0);
          digitalWrite(rcv_dir[0], HIGH);
          analogWrite(rcv_pwm[0], 255*0);

    
    // for (int i = 0; i<2; i++){
    //     digitalWrite(rcv_dir[i], HIGH);
    //     analogWrite(rcv_pwm[i], 255*0);

    //   }
   
    
  }
  else if (buttons == 4) {
    buttons = 0;
    servo_stop.write(90); 
    // feed_pid_timer.begin(feed_pid,100000000);
    // rcv_pid_timer.begin(rcv_pid,100000000);

    Serial.println("DRIBBLING");
    // rollers();
    // dcv_control();
    digitalWrite(dribbling_arm_out,LOW);
    digitalWrite(dribbling_arm_in,HIGH);
    delay(500);
    // digitalWrite(dribbling_arm_out,LOW);
    // digitalWrite(dribbling_arm_in,HIGH);
    digitalWrite(dribbling_arm_out,LOW);
    digitalWrite(dribbling_arm_in,LOW);
    
    servomotion(15,90);
    delay(1000);
    dcv_control();
    delay(50);
    rollers();
    delay(600);
    // pixy.setLamp(1, 1);
    // while (true) {
    //   pixy.ccc.getBlocks();

    //   if (pixy.ccc.numBlocks) {
    //     function();
    //     Serial.println("Detected");
    //     pixy.setLamp(0, 0);
    //     break;
    //   }
    
    // function();
    delay(200);
    servomotion(90,15);
    stoprollers();
    delay(500);
    digitalWrite(dribbling_arm_out,HIGH);
    digitalWrite(dribbling_arm_in,LOW);
    delay(500);
    digitalWrite(dribbling_arm_out,LOW);
    digitalWrite(dribbling_arm_in,LOW);
    servo_stop.write(0);


    // feed_pid_timer.begin(feed_pid,10000);
    // rcv_pid_timer.begin(rcv_pid,10000);
    // feeding
    // sp1 = 3000;
  
  }
}

#include <Encoder.h>
#include <VescUart.h>
#include <Servo.h>
#include <Pixy2I2C.h>
Pixy2I2C pixy;

VescUart UART;

IntervalTimer feed_pid_timer;
IntervalTimer rcv_pid_timer;

int flag_rec=0;
int flag_feed=0;


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
Encoder myEnc_feed(15,14);   // feeding 
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
Encoder myEnc_rcv[2] = {Encoder(40,41), Encoder(39,38)};     //right, left

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
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);


  // dribbling
  pixy.init();
  pixy.setLamp(1,1);//to turn on pixy led

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

  feed_pid_timer.begin(feed_pid, 10000);
  rcv_pid_timer.begin(rcv_pid, 10000);

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

  pid1 = constraint(pid1, -255*64, 255*64);

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
    Rprev_err[i] = Rerr[i];

    Rpid[i] = constraint(Rpid[i], -255*64, 255*64);

    digitalWrite(rcv_dir[i], (Rpid[i] <= 0 ? LOW : HIGH));
    analogWrite(rcv_pwm[i], abs(Rpid[i]));

  }

  // Serial.printf(" sp1:%f", sp[0]);
  // Serial.printf(" ticks1:%d", myEnc[0].read());
  // Serial.printf(" sp2:%f", sp[1]);
  // Serial.printf(" ticks2:%d\n", myEnc[1].read());

}


int buttons = 0;


void loop() {
  // Serial.println("ok");

  if (Serial2.available() >=  sizeof(buttons)) {
    // Read buttons
    Serial2.readBytes((char*)&buttons, sizeof(buttons));
    Serial.print(" | Buttons: ");
    // buttons=buttons;
    Serial.println(buttons);
  }

  if (buttons == 2 && flag_feed==0) {
    buttons = 0;
    // UART.setRPM(1500);
    sp1=-3500;
    flag_feed=1;
  }
  else if(buttons == 2 && flag_feed==1){
    buttons = 0;
    sp1=0;
    flag_feed=0;
    // UART.setRPM(0);
  }
  else if (buttons == 8 && flag_rec==0) {
    buttons = 0;
    Rsp[0]=5000;
    Rsp[1]=5000;
    flag_rec=1;
  }
  else if(buttons == 8 && flag_rec==1)
  {
    buttons = 0;
    Rsp[0]=0;
    Rsp[1]=0;
    flag_rec=0;
  }
  else if (buttons == 4) {
    buttons = 0;
    servo_stop.write(90); 
    feed_pid_timer.begin(feed_pid,100000000);
    rcv_pid_timer.begin(rcv_pid,100000000);

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
    pixy.setLamp(1, 1);
    while (true) {
      pixy.ccc.getBlocks();

      if (pixy.ccc.numBlocks) {
        function();
        Serial.println("Detected");
        pixy.setLamp(0, 0);
        break;
      }
    }
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


    feed_pid_timer.begin(feed_pid,10000);
    rcv_pid_timer.begin(rcv_pid,10000);
    // feeding
    // sp1 = 3000;
  
  }

}
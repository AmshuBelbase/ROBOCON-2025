#define TX_PIN 8 // Teensy 4.1 Tx pin (Serial2)
#define RX_PIN 7 // Teensy 4.1 Rx pin (Serial2)//Recieving data from another teensy
#include <Encoder.h>
// #include <VescUart.h>
#include <Servo.h>
#include <Pixy2I2C.h>
Pixy2I2C pixy;

IntervalTimer feedPosPid;
IntervalTimer recPosPid;
int constraint(int value, int min_val, int max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

int cpr500RPM=538;
int pwmFeed=19;
int dirFeed=16;
Encoder feedEnc(15,14);

int pwmRecieveR=23;
int dirRecieveR=21;
Encoder encRecieveR(40,41);

int pwmRecieveL=22;
int dirRecieveL=20;
Encoder encRecieveL(39,38);

int pwmRoller=11;
int dirRoller=12;

int servoCam=0;
int servoDribR=1;
int servoDribL=2;
int servoStop=3;

Servo srvCam;
Servo srvDribR;
Servo srvDribL;
Servo srvStop;

int ballPushExtend=27;
int ballPushRetract=26;

int dribblingMechExtend=31;
int dribblingMechRetract=30;

int catchingExtend=28;
int catchingRetract=29;


float kP=150,kI=5;
float kD=1;

int L2_value = 0;
int R2_value = 0;

int turntable_pwm = 18;
int turntable_dir = 17;
int rpm = 0;

void setPosition(int pwm,int dir ,int speed)
{
  digitalWrite(dir,speed<0?LOW:HIGH);
  analogWrite(pwm,abs(speed));
}


float err_feed=0,der_feed=0,integ_feed=0,lastError_feed=0;
float speed_feed=0;
float ap_count_feeder=0;
float sp_angle_feeder=0;
float sp_count_feeder=0;
void feedPid()
{
ap_count_feeder=feedEnc.read();
sp_count_feeder=sp_angle_feeder*cpr500RPM/360.0;

err_feed=sp_count_feeder-ap_count_feeder;
der_feed=(err_feed-lastError_feed)/0.01;
integ_feed=integ_feed+(err_feed-lastError_feed)*0.01;
lastError_feed=err_feed;

speed_feed=kP*err_feed+kI*integ_feed+kD*der_feed;
speed_feed=constraint(speed_feed,-255*64,255*64);

setPosition(pwmFeed,dirFeed,int(speed_feed));
}


float apL=0.0;
float sp_angleL=0.0;

float apR=0.0;
float sp_angleR=0.0;

float spL=0;
float errL=0,derL=0,integL=0;
float speed_mL=0;
volatile float last_errL=0;

float spR=0;
float errR=0,derR=0,integR=0;
float speed_mR=0;
volatile float last_errR=0;
void posPid()
{
apL=encRecieveL.read();
spL=sp_angleL*cpr500RPM/360.0;

errL=spL-apL;
derL=(errL-last_errL)/0.01;
integL=integL+(errL-last_errL)*0.01;
last_errL=errL;

speed_mL=kP*errL+kI*integL+kD*derL;



apR=encRecieveR.read();
spR=sp_angleR*cpr500RPM/360.0;

errR=spR-apR;
derR=(errR-last_errR)/0.01;
integR=integR+(errR-last_errR)*0.01;
last_errR=errR;

speed_mR=kP*errR+kI*integR+kD*derR;

speed_mR=constraint(speed_mR,-225*64,225*64);
speed_mL=constraint(speed_mL,-225*64,225*64);

setPosition(pwmRecieveL,dirRecieveL,int(speed_mL));
setPosition(pwmRecieveR,dirRecieveR,int(speed_mR));
}





int flag_rec=0;
int flag_feed=0;
int flag = 0;




void piston(int i ,bool state)//state=1-->extend;state=0-->retract!!!PASS -1 to both ARG for deafult
{
  switch(i)
  {
    case 1:
      if(state==0)
      {
        digitalWrite(ballPushExtend,LOW);
        digitalWrite(ballPushRetract,HIGH);
        delay(500);
        digitalWrite(ballPushExtend,LOW);
        digitalWrite(ballPushRetract,LOW);
      }
      else if(state==1)
      {
        digitalWrite(ballPushExtend,HIGH);
        digitalWrite(ballPushRetract,LOW);
        delay(500);
        digitalWrite(ballPushExtend,LOW);
        digitalWrite(ballPushRetract,LOW);
      }
      break;
    case 2:
      if(state==0)
      {
        digitalWrite(dribblingMechExtend,LOW);
        digitalWrite(dribblingMechRetract,HIGH);
        delay(500);
        digitalWrite(dribblingMechExtend,LOW);
        digitalWrite(dribblingMechRetract,LOW);
      }
      else if(state==1)
      {
        digitalWrite(dribblingMechExtend,HIGH);
        digitalWrite(dribblingMechRetract,LOW);
        delay(500);
        digitalWrite(dribblingMechExtend,LOW);
        digitalWrite(dribblingMechRetract,LOW);
      }
      break;
    case 3:
      if(state==0)
      {
        digitalWrite(catchingExtend,LOW);
        digitalWrite(catchingRetract,HIGH);
        delay(500);
        digitalWrite(catchingExtend,LOW);
        digitalWrite(catchingRetract,LOW);
      }
      else if(state==1)
      {
        digitalWrite(catchingExtend,HIGH);
        digitalWrite(catchingRetract,LOW);
        delay(500);
        digitalWrite(catchingExtend,LOW);
        digitalWrite(catchingRetract,LOW);
      }
      break;
      default:
        digitalWrite(ballPushExtend,HIGH);
        digitalWrite(ballPushRetract,LOW);
        delay(500);
        digitalWrite(ballPushExtend,LOW);
        digitalWrite(ballPushRetract,LOW);

        digitalWrite(dribblingMechRetract,HIGH);
        digitalWrite(dribblingMechExtend,LOW);
        delay(500);
        digitalWrite(dribblingMechRetract,LOW);
        digitalWrite(dribblingMechExtend,LOW);

        digitalWrite(catchingRetract,HIGH);
        digitalWrite(catchingExtend,LOW);
        delay(500);
        digitalWrite(catchingRetract,LOW);
        digitalWrite(catchingExtend,LOW);        



  }
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {

    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

}




    

void setup() {
  // pixy.init();
  Serial.begin(2000000);
  Serial2.begin(115200);
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);

  pinMode(turntable_pwm, OUTPUT);
  pinMode(turntable_dir, OUTPUT);

  digitalWrite(turntable_dir, LOW);
  analogWrite(turntable_pwm, 0);

  analogWriteResolution(14);
  analogWriteFrequency(0, 9000);

  int servoCam=0;
  int servoDribR=1;
  int servoDribL=2;
  int servoStop=3;

  srvCam.attach(servoCam);// Servo srvCam;
  srvDribR.attach(servoDribR);// Servo srvDribR;
  srvDribL.attach(servoDribL);// Servo srvDribL;
  srvStop.attach(servoStop);// Servo srvStop;

  srvDribR.write(180-15);
  srvDribL.write(15);

  pinMode(dirFeed,OUTPUT);
  pinMode(dirRecieveR,OUTPUT);
  pinMode(dirRecieveL,OUTPUT);
  pinMode(dirRoller,OUTPUT);

  pinMode(ballPushExtend,OUTPUT);
  pinMode(ballPushRetract,OUTPUT);
  pinMode(dribblingMechExtend,OUTPUT);
  pinMode(dribblingMechRetract,OUTPUT);
  pinMode(catchingExtend,OUTPUT);
  pinMode(catchingRetract,OUTPUT);

  //Initializing all the piston position
  piston(-1,-1);

  feedPosPid.begin(feedPid,10000);
  recPosPid.begin(posPid,10000);

  feedPosPid.priority(0);
  recPosPid.priority(1);
}
void servoEase(int Spos, int Epos, int duration) {
 int lastTime=millis();
 float x=0;
 float y=0;
  while(millis()-lastTime<=duration)
  {
      x=mapFloat(millis()-lastTime,0,duration,0,1);
      y = x < 0.5 ? (1 - sqrt(1 - pow(2 * x, 2))) / 2 : (sqrt(1 - pow(-2 * x + 2, 2)) + 1) / 2;
      int servoPosL =int( mapFloat(y, 0, 1, Spos, Epos));
      int servoPosR = 180 - servoPosL;
      Serial.printf("Servo Left: %d",servoPosL);
      Serial.printf("Servo Left: %d",servoPosL);

      srvDribR.write(servoPosR);
      srvDribL.write(servoPosL);

  }
}

int psAxis[6];
int buttons;
int a=0;


void loop() {
  // Serial.println("ok");
  // put your main code here, to run repeatedly:
  if (Serial2.available() >= sizeof(psAxis) + sizeof(buttons)+1 && Serial2.read()==0xAA) {
      // Read array
      Serial.println("rcvd");
      Serial2.readBytes((char*)psAxis, sizeof(psAxis));
      L2_value = psAxis[3];
      R2_value = psAxis[4];

      // Read buttons
      Serial2.readBytes((char*)&buttons, sizeof(buttons));

      // Print received data
      Serial.print("Received Array: ");
      for (int i = 0; i < 6; i++) {
        psAxis[i]=psAxis[i];
          Serial.print(psAxis[i]);
          Serial.print(" ");
      }
      Serial.print(" | Buttons: ");
      buttons=buttons;
      Serial.println(buttons);
  }
  // Serial.printf("L2: %d, R2: %d, RPM: %d, Button: %d\n", L2_value*16, R2_value, rpm, buttons);
  // Serial.printf("L2:%d\n", L2_value);
  if (buttons == 131136 && L2_value !=0){       // right
    digitalWrite(turntable_dir, HIGH);
    analogWrite(turntable_pwm, L2_value*16);
  }
  else if(buttons == 524352 && L2_value !=0){    // left
    digitalWrite(turntable_dir, LOW);
    analogWrite(turntable_pwm, L2_value*16);
  }
  else if (buttons == 64 && L2_value != 0){
    digitalWrite(turntable_dir, LOW);
    analogWrite(turntable_pwm, 0);
  }
  else if (buttons == 1 && flag == 0){
    buttons = 0;
    srvCam.write(0);
    flag = 1;
  }
  else if (buttons == 1 && flag == 1){
    buttons = 0;
    srvCam.write(00);
    flag = 0;
  }
  else if (buttons == 2 && flag_feed==0) {
    // {servomotion(0,150);
    // UART.setRPM(1500);
    Serial.println("Cross pressed");
    sp_angle_feeder=-3650;
    flag_feed=1;
    buttons=0;
  }
  else if(buttons == 2 && flag_feed==1){
    sp_angle_feeder=0;
    flag_feed=0;
    // UART.setRPM(0);
    buttons=0;
  }
  else if (buttons == 8 && flag_rec==0) {

    sp_angleR=4650;
    sp_angleL=4650;
    flag_rec=1;
    srvStop.write(0);
    Serial.println("Triangle pressed0");
    buttons=0;
  }
  else if(buttons == 8 && flag_rec==1){
    sp_angleR=0;
      sp_angleL=0;
      flag_rec=0;
       Serial.println("Triangle pressed1");  
       buttons=0;
  }

   /*                            0   1.      __________1

                                4     5       ___________2   

                            2             3.  _____________3      


*/
   else if (buttons == 4) {
    Serial.println("Circle pressed");    // servomotion(150,0);

    srvStop.write(90); 
    feedPosPid.begin(feedPid,100000000);
    recPosPid.begin(posPid,100000000);
    // pid_timer.begin(pid, 100000000);  // Stop the interrupt timer
    //Serial.println("PID Timer Stopped");
    Serial.println("DRIBBLING");
    // rollers();
    // dcv_control();
    piston(2,1);
    servoEase(15,100,1000);
    delay(1000);
    piston(1,0);
    delay(50);
    digitalWrite(dirRoller,HIGH);
    analogWrite(pwmRoller,225*64);
    delay(600);
    pixy.setLamp(1, 1);
    while (true) {
      pixy.ccc.getBlocks();

      if (pixy.ccc.numBlocks) {
        piston(3,1);
        
        delay(500);
        piston(3,0);
        Serial.println("Detected");
        pixy.setLamp(0, 0);
        break;
      }


      
    }
    // function();
    delay(200);
    servoEase(100,15,1000);
    digitalWrite(dirRoller,HIGH);
    analogWrite(pwmRoller,0);
    delay(500);
    srvStop.write(0); 
    piston(2,0);
    piston(1,1);
    // Serial.println("Restarting PID Timer");
    // pid_timer.begin(pid, 75000);
    // Serial.println("PID Timer Restarted");
    feedPosPid.begin(feedPid,10000);
    recPosPid.begin(posPid,10000);
    // feeding
    // sp_angle_feeder=-3000;
    // feed_pid_timer.begin(calcPID, 10000);
    // if (err1 == 0) {
    //   feed_pid_timer.end();
    buttons=0;
    
   }
}
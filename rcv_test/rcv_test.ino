#include "USBHost_t36.h"
#include <Encoder.h>
// #include <cstdlib>
// #include "Easing.h"
IntervalTimer pos_pid_timer;
Encoder encR(9,10);
Encoder encL(6,7);

// Easing mL;
int shooter_rotation_pwmL=19;
int shooter_rotation_dirL=17;

int shooter_rotation_pwmR=18;
int shooter_rotation_dirR=16;

float shooter_rotation_cpr=538;
float kP=150,kI=5,kD=1;

int shooting_angleL=0;
int shooting_angleR=0;

float apL=0.0;
float sp_angleL=0.0;

float apR=0.0;
float sp_angleR=0.0;

// int pos_address=(int)malloc(sizeof(int));//allocated memory for a pointer which will store the value that should be calculated by 
//char dir=0;

USBHost myusb;
JoystickController joystick1(myusb);
// BluetoothController bluet(myusb, true, "0000");
BluetoothController bluet(myusb);   // version assumes it already was paireduint32_t buttons_prev = 0;

uint32_t buttons_prev = 0;
uint32_t buttons;

int psAxis[64];
int psAxis_prev[64];
bool first_joystick_message = true;


void setup()
{
pinMode(13,OUTPUT);
digitalWrite(13,HIGH);
Serial.begin(2000000);
pinMode(shooter_rotation_pwmL,OUTPUT);
pinMode(shooter_rotation_dirL,OUTPUT);

pinMode(shooter_rotation_pwmR,OUTPUT);
pinMode(shooter_rotation_dirR,OUTPUT);

analogWriteResolution(14);

pos_pid_timer.begin(pos_pid,10000);

}
void setPosition(int pwm,int dir ,int speed)
{
digitalWrite(dir,speed<0?LOW:HIGH);
analogWrite(pwm,abs(speed));
}

float spL=0;
float errL=0,derL=0,integL=0;
float speed_mL=0;
volatile float last_errL=0;

float spR=0;
float errR=0,derR=0,integR=0;
float speed_mR=0;
volatile float last_errR=0;


void pos_pid()
{
apL=encL.read();
spL=sp_angleL*shooter_rotation_cpr/360.0;

errL=spL-apL;
derL=(errL-last_errL)/0.01;
integL=integL+(errL-last_errL)*0.01;
last_errL=errL;


speed_mL=kP*errL+kI*integL+kD*derL;



apR=encR.read();
spR=sp_angleR*shooter_rotation_cpr/360.0;

errR=spR-apR;
derR=(errR-last_errR)/0.01;
integR=integR+(errR-last_errR)*0.01;
last_errR=errR;

speed_mR=kP*errR+kI*integR+kD*derR;




// Serial.printf("speed: %f   ",speed_m);
// Serial.printf("Kp: %f.   Ki: %f.    Kd: %f   ",kp,ki,kd);
Serial.print("SpL:");
Serial.print(sp_angleL);
Serial.print("     ApL:");
Serial.println(int((apL*360.0/shooter_rotation_cpr)));
//       // Serial.println(enc1.read());
setPosition(shooter_rotation_pwmL,shooter_rotation_dirL,int(speed_mL));

Serial.print("SpR:");
Serial.print(sp_angleR);
Serial.print("     ApR:");
Serial.println(int((apR*360.0/shooter_rotation_cpr)));
      // Serial.println(enc1.read());
setPosition(shooter_rotation_pwmR,shooter_rotation_dirR,int(speed_mR));



}
int lastTime=0,final_pos=0;

void loop()
{

//drive(5000);

    if (millis() - lastTime > 1000) {
      noInterrupts();

     if (Serial.available() > 0) {
       String input = Serial.readString();//0000,0000,0000...ip,fp,dur(ms)
      // Serial.println(input.length());
       if (input.length() <= 20) {
         //int ip=(input.substring(0, 1)).charAt(0);
        //  int ip=(input.substring(0,4)).toInt();
        //  int fp=(input.substring(5,9)).toInt();
        //  int dur=(input.substring(10,14)).toInt();
         sp_angleL=-1*input.substring(0).toInt();
         sp_angleR=-1*sp_angleL;
        
         //sp_angleL=int(shooting_angleL*6.25);
        
        // mL.easeInOutExpo(ip,fp,dur,pos_address);
        //  sp_angleR=*pos_address;
        //  sp_angleL=-1*sp_angleR;
        
       }
     } 

        interrupts();
      lastTime = millis();
    }


  //ps4_input();
}
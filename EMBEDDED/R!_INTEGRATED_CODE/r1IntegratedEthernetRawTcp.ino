#include "USBHost_t36.h"
#include <Encoder.h>
#include <VescUart.h>
#include <ClientServerEthernet.h>
VescUart UART;

IntervalTimer pid_timer;
IntervalTimer turntable_timer;

#pragma pack(push, 1)
struct ControllerData { 
    int32_t axis[4]; 
    int16_t r1;
    int16_t cross;
    int16_t circle;
    int16_t triangle; 
    int16_t turn_pwm;
    int16_t bldc_pwm;
};
#pragma pack(pop)

int axis[4]= {128};
int r1;
ControllerData jetdata;
ClientServerEthernet<ControllerData> con;

Encoder myEnc[3] = { Encoder(22, 23), Encoder(20, 21), Encoder(18, 17) };

bool flag = 0;
bool flag_timer=0;
int lastTime=0;

int x, y, leftX;  
// int PWM[3] = { 4, 5, 3 };
// int DIR[3] = { 6, 7, 2 };
int PWM[3] = { 4, 6, 10 };
int DIR[3] = { 3, 5, 11 };


long currentCounts[3] = {0,0,0};
volatile long lastCount[3] = {0,0,0};      
volatile double rpm[3] = {0,0,0};          // Stores the calculated RPM
long positionChange[3] = {0,0,0};

//pid constants
float kp[3] = {9.0, 9.0, 9.0};
float ki[3] = {165.0, 165.0, 165.0};
float kd[3] = {0.5, 0.5, 0.5}; 

volatile float sp[3]={0,0,0};
volatile float new_sp[3]={0,0,0};
float pid[3] = {0.0, 0.0, 0.0};
float err[3] = {0.0, 0.0, 0.0};
float prev_err[3] = {0.0, 0.0, 0.0};
float integ[3] = {0.0, 0.0, 0.0};
float der[3] = {0.0, 0.0, 0.0}; 

float max_rpm = 500;

float rad, mag, new_mag;

// float max_rpm = 100;

int max_xy = 100;
float max_mag = sqrt(max_xy*max_xy + max_xy*max_xy);

// IntervalTimer timer; // Timer object for periodic execution

#include <math.h>
#define M_PI 3.14159265358979323846

double easeInOutSine(double x) {
  return -(cos(M_PI * x) - 1) / 2;
}

float easeInSine(float x) {
  return 1 - cos((x * M_PI) / 2);
}

float easeInQuad(float x) {
  return x*x;
}

double easeInOutCirc(double x) {
  if (x < 0.5)
      return (1 - std::sqrt(1 - std::pow(2 * x, 2))) / 2;
  else
      return (std::sqrt(1 - std::pow(-2 * x + 2, 2)) + 1) / 2;
}


float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {

  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

}


//////// drive


// turntable 
int direc_pin=9;
int pwm_pin=8;
int turn_pwm=0;
int turn_pwm_temp=0;
int bldc_pwm=0;
int bldc_pwm_temp=0;

int maxpwm=230;
int count_not_detected = 0;

// feeding
volatile int feeder_pwm=24;
volatile int feeder_dir=12;

// bldc
int orpm=0;
int nrpm=0;


void setup() {

  vector<int> client_ip = {192, 168, 1, 101}; // IP address of this device (client)
  vector<int> server_ip = {192, 168, 1, 102}; // IP address of the server to communicate with
  vector<int> subnet_mask = {255, 255, 255, 0}; // Subnet mask for the network

  con = ClientServerEthernet<ControllerData>(client_ip, subnet_mask, server_ip, &jetdata);

  Serial.begin(9600);

  // bldc
  Serial1.begin(115200);  
  while (!Serial1) {;}
  UART.setSerialPort(&Serial1);

  // teensy led
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  // turntable
  pinMode(direc_pin,OUTPUT);
  pinMode(pwm_pin,OUTPUT);

  analogWrite(pwm_pin, 0);
  digitalWrite(direc_pin, LOW);

  // feeding
  pinMode(feeder_pwm, OUTPUT);
  pinMode(feeder_dir, OUTPUT);

  analogWrite(feeder_pwm, 0);
  digitalWrite(feeder_dir, LOW);


  // drive
  // Motor control pins setup
  for (int i = 0; i < 3; i++) {
    pinMode(PWM[i], OUTPUT);
    pinMode(DIR[i], OUTPUT);
  }

  // Initialize motor to stop
  for (int i = 0; i < 3; i++) {
    analogWrite(PWM[i], 0);
    digitalWrite(DIR[i], LOW);
  }

  analogWriteResolution(14);
  analogWriteFrequency(0, 9000);

  pid_timer.priority(1);
  turntable_timer.priority(0);


  turntable_timer.begin(turntable, 10000);
  // anant.priority(0);
  // ps4Timer.begin(ps4, 10000);
  // ps4Timer.priority(0);
  // anant.begin(force_start , 100000);
  // pid_timer.begin(drive_pid, 75000);


}

// void force_start(){
//   // Ethernet.begin();
//   // Ethernet.setLocalIP(myIP);
//   udp.begin(port);
// }
// uint32_t button=0;
// uint32_t prev_button;

// void ps4(){
//   int packetSize = udp.parsePacket();
//   if(packetSize){
//     char packet[64];

//     int len = udp.read(packet , sizeof(packet) - 1);
//     if(len>0){
//       packet[len] = '\0';
//       Serial.print("Recieved: ");
//       Serial.println(packet);
//       int parsed = sscanf(packet , "%d,%d,%d,%d,%d" , &axis[0] , &axis[1] , &axis[2] , &axis[3] , &r1);
//       if(parsed==5){
//         Serial.printf("LX: %d  LY: %d  RX: %d  RY: %d\n", axis[0], axis[1], axis[2], axis[3] , r1);
//       }
//       else{
//         Serial.println("Packet Parsing Error");
//       }
//     }
//   }
//   else{
//     Serial.println("Message Sending Error");
//   }
// }


// drive
void drive_pid(){

  // input();
    // Handle USB host tasks

    // Left Stick values (axes 0 and 1)
    int leftStickX = axis[0];
    leftX = map(leftStickX, 0, 255, -100, 100);

    // Right Stick values (axes 2 and 5)
    int rightStickX = axis[2];
    x = map(rightStickX, 0, 255, -100, 100);
    // x=0;

    int rightStickY = axis[3];
    y = map(rightStickY, 0, 255, -100, 100);

    // round off 
    // x = round(x/10)*10;
    // y = round(y/10)*10;
    // leftX = round(leftX/10)*10;

    //to ignore small joystick values
    if (abs(x) < 5) x = 0;
    if (abs(y) < 5) y = 0;
    if (abs(leftX) < 5) leftX = 0;


    mag = sqrt(x * x + y * y);

    if (x == 0){
      rad = 1.5708;
    }else{
      rad = atan(fabs(y) / fabs(x));
    }

    new_mag = mapFloat(easeInOutCirc(mapFloat(mag, 0, max_mag, 0, 1)), 0, 1, 0, max_mag);
    // Serial.printf(" new_mag:%f",new_mag);

    int newx = cos(rad) * new_mag;
    int newy = sin(rad) * new_mag;

    // Serial.printf(" newx:%f\n",newx);
    // Serial.printf(" newy:%f",newy);

    if (x < 0) newx *= -1;
    if (y < 0) newy *= -1;


    if (r1 && (flag == 0 && flag_timer == 1)){
      max_rpm = 500;
      digitalWrite(13, HIGH);
      flag = 1;
      flag_timer = 0;
      Serial.println("500RPM");
    }
    else if (r1 && (flag == 1 && flag_timer == 1)){
      max_rpm = 250;
      digitalWrite(13, LOW);
      flag = 0;
      flag_timer = 0;
    Serial.println("250RPM");

    }
          // Serial.println(lastTime);

    // Serial.print(flag_timer);
    if(millis()-lastTime>=1000)
    {
      flag_timer = 1;
      lastTime=millis();
      
    }

    
    // Serial.printf(" x:%d\n",x);
    // Serial.printf(" y:%d",y);
    // Serial.printf(" left x:%d",leftX);

  sp[0] = ((x) * (-0.67) + (y) * 0 + (leftX) * (-0.33));        
  sp[1] = ((x) * (0.33) + (y) * (-0.57) + (leftX) * (-0.33)); 
  sp[2] = ((x) * (0.33) + (y) * (0.57) + (leftX) * (-0.33)); 

  new_sp[0] = ((newx) * (-0.67) + (newy) * 0 + (leftX) * (-0.33));        
  new_sp[1] = ((newx) * (0.33) + (newy) * (-0.57) + (leftX) * (-0.33)); 
  new_sp[2] = ((newx) * (0.33) + (newy) * (0.57) + (leftX) * (-0.33)); 

  sp[0] = map(sp[0], -72, 72, -max_rpm, max_rpm);
  sp[1] = map(sp[1], -72, 72, -max_rpm, max_rpm);
  sp[2] = map(sp[2], -72, 72, -max_rpm, max_rpm);

  new_sp[0] = map(new_sp[0], -80, 80, -max_rpm, max_rpm);
  new_sp[1] = map(new_sp[1], -80, 80, -max_rpm, max_rpm);
  new_sp[2] = map(new_sp[2], -80, 80, -max_rpm, max_rpm);

  // Serial.printf(" sp1:%0.2f", sp[0]);
  // Serial.printf(" sp2:%0.2f", sp[1]);
  // Serial.printf(" sp3:%0.2f", sp[2]);

  Serial.printf(" newsp1:%0.2f", new_sp[0]);
  Serial.printf(" newsp2:%0.2f", new_sp[1]);
  Serial.printf(" newsp3:%0.2f", new_sp[2]);


  // Calculate RPM
  for (int i=0; i<3; i++){
    currentCounts[i] = myEnc[i].read();
    positionChange[i] = currentCounts[i] - lastCount[i];
    rpm[i] = (positionChange[i] / 1300.0) * (60 * (1000.0 / 75));
    lastCount[i] = currentCounts[i];

  }
  Serial.printf(" rpm1:%f", rpm[0]);
  Serial.printf(" rpm2:%f", rpm[1]);
  Serial.printf(" rpm3:%f\n", rpm[2]);

  //PID Control
  for (int i=0; i<3; i++){
    err[i] = new_sp[i] - rpm[i];
    integ[i] = integ[i] + (err[i]*0.075);   
    der[i] = (err[i]-prev_err[i])/0.075;

    pid[i] = (kp[i]*err[i]) + (ki[i]*integ[i]) + (kd[i]*der[i]);
    prev_err[i] = err[i];

    digitalWrite(DIR[i], (pid[i] <= 0 ? LOW : HIGH));
    analogWrite(PWM[i], abs(pid[i]));

    pid[i] = constrain(pid[i], -16383, 16383);

  }
  // for (int i = 0; i < 4; i++){
  //   axis[i] =128;
  // }

}
//////drive



// check if hoop is not detected for continuos 5 times (frames), if not detected, stop the motor.
void turntable(){
    if(turn_pwm==1){
      count_not_detected++;
    if(count_not_detected == 5){
      count_not_detected = 0;
      digitalWrite(direc_pin,LOW);
      analogWrite(pwm_pin,0);
    }
  }
  // 1 represents the hoop is in center
  else if(turn_pwm == 0){
    count_not_detected = 0;
    digitalWrite(direc_pin,LOW);
    analogWrite(pwm_pin,0);
  }
  // if not in center rotate accordingly
  else if(turn_pwm!=0){
    count_not_detected = 0;
    digitalWrite(direc_pin,(turn_pwm>0)?LOW:HIGH);
    analogWrite(pwm_pin,abs(turn_pwm*32));
  }
  // stop at any extreme (undetermined) conditions
  else{
    digitalWrite(direc_pin,LOW);
    analogWrite(pwm_pin,0);
  }
}

//////////////// turntable 




void loop() {
  con.getData(true);
  con.MaintainConnection();
  for(int i=0 ; i<4 ;i++){
    axis[i] = jetdata.axis[i];
  }
  r1 = jetdata.r1;


  if(jetdata.triangle){
    //feed up
    digitalWrite(feeder_dir , LOW);
    analogWrite(feeder_pwm , 255*48);
  }else if(jetdata.cross){
    //feed down
    digitalWrite(feeder_dir , HIGH);
    analogWrite(feeder_pwm , 255*48);
  }else{
    digitalWrite(feeder_dir , LOW);
    analogWrite(feeder_pwm , 255*0);
  }

  // BLDC
  if(jetdata.circle){
    // nrpm = bldc_pwm;
    Serial.println("BLDC");
    Serial.println(orpm);
    Serial.println(nrpm);

    while(orpm < nrpm){
      orpm=orpm+((nrpm-orpm)/700)+((nrpm-orpm)%700);
      Serial.println(orpm);
      UART.setRPM(orpm);
      delay(50); 
    }
    while(orpm > nrpm){
      orpm=orpm-((orpm-nrpm)/700)-((orpm-nrpm)%700);
      Serial.println(orpm);
      UART.setRPM(orpm);
      delay(50); 
    }  
    UART.setRPM(orpm);
  } 
  else
  {UART.setRPM(0);}

  // if(haty == 1 && haty!=last_haty){
  //   nrpm += 50*7;
  //   Serial.println("+50"); 
  // }


  // if(haty == -1 && haty!=last_haty){
  //   nrpm -= 50*7;
  //   Serial.println("-50"); 
  // }

  // last_r2 = r2;
  // last_triangle = triangle;
  // last_cross = cross;
  // last_haty = haty;

  // circle=0; triangle=0; cross=0;



  
}
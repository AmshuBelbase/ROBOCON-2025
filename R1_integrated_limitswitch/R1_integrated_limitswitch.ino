#include "USBHost_t36.h"
#include <Encoder.h>
#include <VescUart.h>
VescUart UART;

IntervalTimer pid_timer;
IntervalTimer turntable_timer;
IntervalTimer data_timer;
// IntervalTimer anant;
// IntervalTimer ps4Timer;

#include <QNEthernet.h>

using namespace qindesign::network;

EthernetServer server(8000);  
String msgRecieved = "";
String tempBuffer = "";

// drive
// Encoder myEnc[3] = { Encoder(36, 33), Encoder(38, 37), Encoder(40, 39) };
Encoder myEnc[3] = { Encoder(22, 23), Encoder(20, 21), Encoder(18, 17) };

//PS4 connection 

int axis[4] = {128};
int r1 = 0;

// USBHost myusb;   // initializes and manages the USB host port, enabling Teensy to detect and commuincate with USB bluetooth dongle
// USBHIDParser hid1(myusb);  //works behind the scenes to parse HID data that comes from the PS4 controller, such as joystick movements and button presses.
// JoystickController joystick1(myusb);
// // BluetoothController bluet(myusb, true, "0000"); 
// BluetoothController bluet(myusb);   // Version does pairing to device
// uint32_t buttons;
// bool isJoystick = 0;
bool flag = 0;
bool flag_timer=0;
int lastTime=0;
EthernetClient client;

//coordinates of joystick (x,y -> right joystick; leftX -> left joystick)
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
int bldc_pwm=0;

int maxpwm=230;
int count_not_detected = 0;

// feeding
volatile int feeder_pwm=24;
volatile int feeder_dir=12;

bool limitSwitchState[2] = {false};         
int limitSwitch[2]={41,40};  
bool flag_button = 0;

int triangle=0, circle=0, cross=0;
// int last_triangle, last_cross;

// bldc
int orpm=0;
int nrpm=0;


void setup() {

  Serial.begin(9600);

  // bldc
  Serial1.begin(115200);  
  while (!Serial1) {;}
  UART.setSerialPort(&Serial1);

  // teensy led
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);


  // Ethernet.begin();
  // Ethernet.setLocalIP(myIP);
  // udp.begin(port);

  // Serial.print("UDP Receiver ready at IP: ");
  // Serial.println(Ethernet.localIP());


  IPAddress ip(192, 168, 1, 101);  
  IPAddress subnet(255, 255, 255, 0);

  Ethernet.begin();
  Ethernet.setLocalIP(ip);
  Ethernet.setSubnetMask(subnet);

  server.begin();

  Serial.print("TCP Server chalu at: ");
  Serial.println(Ethernet.localIP());

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

  // data_timer.priority(2);
  // data_timer.begin(data_recieve , 10000);

  turntable_timer.begin(turntable, 10000);

  pinMode(limitSwitch[0],INPUT_PULLUP);
  pinMode(limitSwitch[1],INPUT_PULLUP);

  limitSwitchState[1] = digitalRead(limitSwitch[1]); 
  limitSwitchState[0] = digitalRead(limitSwitch[0]);

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

  // sp[0] = ((x) * (-0.67) + (y) * 0 + (leftX) * (-0.33));        
  // sp[1] = ((x) * (0.33) + (y) * (-0.57) + (leftX) * (-0.33)); 
  // sp[2] = ((x) * (0.33) + (y) * (0.57) + (leftX) * (-0.33)); 

  new_sp[0] = ((newx) * (-0.67) + (newy) * 0 + (leftX) * (-0.33));        
  new_sp[1] = ((newx) * (0.33) + (newy) * (-0.57) + (leftX) * (-0.33)); 
  new_sp[2] = ((newx) * (0.33) + (newy) * (0.57) + (leftX) * (-0.33)); 

  // sp[0] = map(sp[0], -72, 72, -max_rpm, max_rpm);
  // sp[1] = map(sp[1], -72, 72, -max_rpm, max_rpm);
  // sp[2] = map(sp[2], -72, 72, -max_rpm, max_rpm);

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



void data_recieve(){

  if (!client || !client.connected()) {
    // client.stop();
    client = server.available();  
    if (client) {
      Serial.println("Client aagya");  
      pid_timer.begin(drive_pid, 75000);
    }
  }

  // If client is connected and there's data available

  if (client && client.connected() && client.available()) {
    // pid_timer.begin(drive_pid, 75000);
    while (client.available()) {

      char c = client.read();   
      if(c=='\n'){
        msgRecieved = tempBuffer;
        tempBuffer = "";
        Serial.print("Recieved: ");
        Serial.println(msgRecieved);
        int parsed = sscanf(msgRecieved.c_str() , "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d" , &axis[0] , &axis[1] , &axis[2] , &axis[3] , &r1, &cross, &circle, &triangle, &turn_pwm, &bldc_pwm);
        if(parsed==10){ 
          nrpm = bldc_pwm*7;
          // nrpm = constrain(nrpm, 0, 4500*7);
        Serial.printf("LX: %d  LY: %d  RX: %d  RY: %d r1: %d turn_pwm: %d bldc_pwm: %d\n", axis[0], axis[1], axis[2], axis[3] , triangle, turn_pwm, bldc_pwm);
        }
        else{
        Serial.println("Packet Parsing Error");
        }
      }
      else{
        tempBuffer += c;       
      }
    }
  }
}
// check if hoop is not detected for continuos 5 times (frames), if not detected, stop the motor.

// turntable 
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
    analogWrite(pwm_pin,abs(turn_pwm*64));
  }
  // stop at any extreme (undetermined) conditions
  else{
    digitalWrite(direc_pin,LOW);
    analogWrite(pwm_pin,0);
  }
}

//////////////// turntable 



void loop() {
  // Serial.println("ok");
  data_recieve();
  
  limitSwitchState[1] = digitalRead(limitSwitch[1]);
  limitSwitchState[0] = digitalRead(limitSwitch[0]);

  // Serial.print("Up:");
  // Serial.print(limitSwitchState[1]); 
  // Serial.print("   Down:");
  // Serial.println(limitSwitchState[0]); 

  if (circle && limitSwitchState[0] == 0){
    
    Serial.println("START bldc");
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

    limitSwitchState[1] = digitalRead(limitSwitch[1]);
    limitSwitchState[0] = digitalRead(limitSwitch[0]);

    Serial.print("Going Up");
    Serial.print(limitSwitchState[1]); 
    Serial.print(limitSwitchState[0]); 


    int cnt = 0;
    while(cnt<=5){
      UART.setRPM(orpm);
      digitalWrite(feeder_dir,HIGH);
      analogWrite(feeder_pwm,64*255);
      limitSwitchState[1] = digitalRead(limitSwitch[1]); 
      limitSwitchState[0] = digitalRead(limitSwitch[0]);
      if(limitSwitchState[1] == 0){
        cnt++;
      }else{
        cnt = 0;
      }
    }

    digitalWrite(feeder_dir,LOW);
    analogWrite(feeder_pwm,0);
    Serial.print("Reached Up");
    Serial.print(limitSwitchState[1]); 
    Serial.print(limitSwitchState[0]); 


    
    // Serial.println("Feed UP");
    // while (limitSwitchState[1] != LOW){
    //   digitalWrite(feeder_dir, HIGH);
    //   analogWrite(feeder_pwm, 255*64);
    //   limitSwitchState[1] = digitalRead(limitSwitch[1]);
    // }
    // digitalWrite(feeder_dir, LOW);
    // analogWrite(feeder_pwm, 255*0);
    // flag_button = 1;
  }
  else if (circle){ 
    UART.setRPM(0);

    limitSwitchState[1] = digitalRead(limitSwitch[1]); 
    limitSwitchState[0] = digitalRead(limitSwitch[0]);

    Serial.print("Going Down");
    Serial.print(limitSwitchState[1]); 
    Serial.print(limitSwitchState[0]); 

    while(limitSwitchState[0]!=0){
      digitalWrite(feeder_dir,LOW);
      analogWrite(feeder_pwm,64*255); 
      limitSwitchState[1] = digitalRead(limitSwitch[1]); 
      limitSwitchState[0] = digitalRead(limitSwitch[0]);
      // Serial.print(".");
    }
    digitalWrite(feeder_dir,HIGH);
    analogWrite(feeder_pwm,0);
    Serial.print("Reached Down"); 
    Serial.print(limitSwitchState[1]); 
    Serial.print(limitSwitchState[0]); 

    // Serial.println("STOP bldc");
    // UART.setRPM(0);

    // Serial.println("feed down");
    // while (limitSwitchState[0] != LOW){
    //   digitalWrite(feeder_dir, LOW);
    //   analogWrite(feeder_pwm, 255*64);
    //   limitSwitchState[0] = digitalRead(limitSwitch[0]);
    // }
    // digitalWrite(feeder_dir, LOW);
    // analogWrite(feeder_pwm, 255*0);
    // flag_button = 0;
  }

  // if (Serial.available() > 0) {
  //   String input = Serial.readStringUntil('\n');
  //   input.trim();
  //   nrpm=input.toInt();
                 
  // }
  

  // FEEDER 

  else if(cross){
    //feed down
    digitalWrite(feeder_dir , LOW);
    analogWrite(feeder_pwm , 255*48);
  }else if(triangle){
    //feed up
    digitalWrite(feeder_dir , HIGH);
    analogWrite(feeder_pwm , 255*48);
  }else{
    digitalWrite(feeder_dir , LOW);
    analogWrite(feeder_pwm , 255*0);
  }

  // BLDC
  // if(circle){
  //   // nrpm = bldc_pwm;
  //   Serial.println("BLDC");
  //   Serial.println(orpm);
  //   Serial.println(nrpm);

  //   while(orpm < nrpm){
  //     orpm=orpm+((nrpm-orpm)/700)+((nrpm-orpm)%700);
  //     Serial.println(orpm);
  //     UART.setRPM(orpm);
  //     delay(50); 
  //   }
  //   while(orpm > nrpm){
  //     orpm=orpm-((orpm-nrpm)/700)-((orpm-nrpm)%700);
  //     Serial.println(orpm);
  //     UART.setRPM(orpm);
  //     delay(50); 
  //   }  
  //   UART.setRPM(orpm);
  // } 
  // else
  // {UART.setRPM(0);}

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
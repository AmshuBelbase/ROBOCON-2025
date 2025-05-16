#include "USBHost_t36.h"
#include <Encoder.h>

IntervalTimer pid_timer;
IntervalTimer anant;
// IntervalTimer ps4Timer;

#include <QNEthernet.h>

using namespace qindesign::network;

EthernetUDP udp;
IPAddress myIP(192, 168, 1, 101); // Match your Jetson's UDP_IP
uint16_t port = 12345;  

// drive
// Encoder myEnc[3] = { Encoder(36, 33), Encoder(38, 37), Encoder(40, 39) };
Encoder myEnc[3] = { Encoder(22, 23), Encoder(20, 21), Encoder(18, 17) };

//PS4 connection 

int axis[4] = {128};
int r1 = 0;
int prev_r1 = 0;


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


void setup() {

  Serial.begin(9600);
  // Serial8.begin(115200);

  // teensy led
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);


  Ethernet.begin();
  Ethernet.setLocalIP(myIP);
  udp.begin(port);

  Serial.print("UDP Receiver ready at IP: ");
  Serial.println(Ethernet.localIP());


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
  anant.priority(0);
  // ps4Timer.begin(ps4, 10000);
  // ps4Timer.priority(0);
  anant.begin(force_start , 100000);
  pid_timer.begin(drive_pid, 75000);


}

void force_start(){
  // Ethernet.begin();
  // Ethernet.setLocalIP(myIP);
  udp.begin(port);
}
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
void drive_pid() {

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
  for (int i = 0; i < 4; i++){
    axis[i] =128;
  }

}
//////drive




void loop() {

  int packetSize = udp.parsePacket();
  if(packetSize){
    char packet[64];

    int len = udp.read(packet , sizeof(packet) - 1);
    if(len>0){
      packet[len] = '\0';
      Serial.print("Recieved: ");
      Serial.println(packet);
      int parsed = sscanf(packet , "%d,%d,%d,%d,%d" , &axis[0] , &axis[1] , &axis[2] , &axis[3] , &r1);
      if(parsed==5){ 
        Serial.printf("LX: %d  LY: %d  RX: %d  RY: %d\n", axis[0], axis[1], axis[2], axis[3] , r1);
      }
      else{
        Serial.println("Packet Parsing Error");
      }
    }
  }
  else{
    Serial.println("Message Sending Error");
  }

}
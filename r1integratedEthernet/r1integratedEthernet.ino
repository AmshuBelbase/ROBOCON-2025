// 128 - SERVO CAMERA
// 64 - DRIBBLING | DIRECT BLDC
// 16 - WITHOUT DRIBBLING ON / OFF
// 8 - FEED UP
// 2 - FEED DOWN
// 4 - BLDC ON / OFF
// 1 - RECEIVING UP / DOWN


#include <VescUart.h>
// #include "USBHost_t36.h"
// #include <Pixy2I2C.h>
#include <Servo.h>
#include <QNEthernet.h>


using namespace qindesign::network;

EthernetUDP udp;
IPAddress myIP(192, 168, 1, 101); // Match your Jetson's UDP_IP
uint16_t port = 12345;  

// Pixy2I2C pixy;
VescUart UART;

int led = 13;
volatile int feeder_pwm=19;
volatile int feeder_dir=17;
// USBHost myusb;
// USBHIDParser hid1(myusb);
// JoystickController joystick1(myusb);
// BluetoothController bluet(myusb, true, "0000");
// BluetoothController bluet(myusb);//, true, "0000");
int turn_pwm = 18;
int turn_dir = 16;
int turn_rpm = 32;

bool flag_rec_r = false;
bool flag_rec_l = false;
bool counter = 1;


int rcv_pwm[2] = {23,22};
int rcv_dir[2] = {21, 20};
int limitSwitch[6] = {14,15,41,40,38,39};
bool limitSwitchState[6] = {false};

int orpm=0;
int nrpm=3000*7;
int index1 = 0;
int safeState = 0;
bool button_4_flag = false; 
bool button_1_flag = false;
bool button_16_flag = false; 


int lx=0, ly=0, rx=0, ry=0,l1=0, r1=0, circle=0, triangle=0, cross=0, square=0,hatx=0,haty=0,l2=0,r2=0;

int last_triangle, last_haty, last_r2,last_cross,last_r1,last_square;

//driblling 
int roller_dir=11;
int roller_pwm=12;
bool flag_roller = 0;

Servo servo_left;
Servo servo_right;
Servo servo_stopper;
Servo servo_cam;
bool servo_flag = false;
int last_btn = 0;

int piston[3][2] = {{26,27} , {31,30} , {28,29}};
/////////dribbling

int lastUpdate = millis();


void setup() {
  // myusb.begin();
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);
  Serial.begin(2000000); 
  // Serial2.begin(115200);
  Serial8.begin(115200);  
  while (!Serial8) {;}
  UART.setSerialPort(&Serial8);

  analogWriteResolution(14);
  analogWriteFrequency(0, 9000);


  Ethernet.begin();
  Ethernet.setLocalIP(myIP);
  udp.begin(port);

  Serial.print("UDP Receiver ready at IP: ");
  Serial.println(Ethernet.localIP());

  // feeding
  pinMode(feeder_pwm, OUTPUT);
  pinMode(feeder_dir, OUTPUT);

  analogWrite(feeder_pwm, 0);
  digitalWrite(feeder_dir, LOW);

  pinMode(turn_pwm , OUTPUT);
  pinMode(turn_dir , OUTPUT);

  analogWrite(turn_pwm, 0);
  digitalWrite(turn_dir, LOW);

  // dribbling
  // pixy.init();

  servo_left.attach(2);
  servo_right.attach(1); 
  servo_stopper.attach(0);
  servo_cam.attach(3);

  servo_left.write(20);
  servo_right.write(180-20);
  servo_stopper.write(0);
  servo_cam.write(0);

  pinMode(roller_dir, OUTPUT);
  pinMode(roller_pwm, OUTPUT);

  for (int i = 0; i < 3; i++){
    for (int j = 0; j < 2; j++){
      pinMode(piston[i][j], OUTPUT);
    }
  }

  pistonControl(0, false);
  pistonControl(1, false);
  pistonControl(2, false);

  // Initializing limit switches 
  for(int i=0;i<6;i++)
  {
    pinMode(limitSwitch[i],INPUT_PULLUP);
  }
  
  // Initializing flag values for feeding and recieving  
  // flag_rec_r = digitalRead(limitSwitch[2]);
  // flag_rec_l = digitalRead(limitSwitch[4]);

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

  
  // teensy led
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
}


void pistonControl(int pistonNum, bool extend){
  digitalWrite(piston[pistonNum][0], extend);
  digitalWrite(piston[pistonNum][1], !extend);
  delay(500);
  digitalWrite(piston[pistonNum][0], LOW);
  digitalWrite(piston[pistonNum][1], LOW);
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


void loop() {
    
  // if (Serial.available() > 0) {   
  //     String input=Serial.readStringUntil('\n');
   
  //     index1 = input.indexOf(',');
  //     safeState = input.substring(0,index1+1).toInt(); 

  //     String tempnrpm = input.substring(index1+1);
  //     nrpm=tempnrpm.toFloat()*7;      
  // }
  
  // TURN TABLE

  // if(safeState < -10){
  //   digitalWrite(turn_dir , HIGH);
  //   analogWrite(turn_pwm , 255*turn_rpm);
  //   lastUpdate = millis();
  // }
  // else if(safeState > 10){
  //   digitalWrite(turn_dir , LOW);
  //   analogWrite(turn_pwm , 255*turn_rpm);
  //   lastUpdate = millis();
  // }else{
  //   digitalWrite(turn_dir , LOW);
  //   analogWrite(turn_pwm , 255*0);
  //   lastUpdate = millis();
  // }

  // if(millis() - lastUpdate >= 10){
  //   safeState = 0;
  // }

  // pS4 Operated
  // if (Serial2.available() >=  sizeof(buttons)) {
  //   // Read buttons
  //   Serial2.readBytes((char*)&buttons, sizeof(buttons));
  //   Serial.print("Buttons: ");
  //   Serial.println(buttons);
  // }


int packetSize = udp.parsePacket();
  if (packetSize) {
    char packet[64];
    int len = udp.read(packet, sizeof(packet) - 1);
    if (len > 0) {
      packet[len] = '\0';

      Serial.print("Received: ");
      Serial.println(packet);
int parsed = sscanf(packet, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                          &lx, &ly, &rx, &ry,
                          &l1, &r1, &circle, &triangle, &cross, &square,&hatx,&haty,&l2,&r2);

      if (parsed == 14) {
        Serial.printf("LX: %d  LY: %d  RX: %d  RY: %d\n", lx, ly, rx, ry);
        Serial.printf("L1: %d  R1: %d  O: %d  T: %d  X: %d  []: %d\n",
                      l1, r1, circle, triangle, cross, square);
      }
    }
  }

  if(haty == 1 && haty!=last_haty){
    nrpm += 50*7;
    Serial.println("+50"); 
  }


  if(haty == -1 && haty!=last_haty){
    nrpm -= 50*7;
    Serial.println("-50"); 
  }



  if( r2 && !servo_flag && r2!=last_r2){ // 0x80 - Button 128 
    for(int i = 0 ; i <60 ; i++){
      servo_cam.write(i);
      delay(15);
    }
    servo_flag = !servo_flag;
  }
  else if( r2 && servo_flag && r2 != last_r2){
    for(int i = 60 ; i >0 ; i--){
      servo_cam.write(i);
      delay(15);
    }
    servo_flag = !servo_flag;
  }




    // if(ps4.available()){
    // buttons = ps4.getButtons();


    if(circle){
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

    // DRIBBLING

    // if(buttons & 0x40){ // 0x10 - Button 64 
    //   pistonControl(1, true);
    //   servomotion(15,92);
    //   servo_stopper.write(90); 
    //   delay(1000);

    //   pistonControl(0, true);
    //   delay(100);
    //   pistonControl(0, false);
    //   delay(50);

    //   rollers();
    //   delay(600);

    //   pixy.setLamp(1, 1);
    //   while (true) {
    //     pixy.ccc.getBlocks();
    //     if (pixy.ccc.numBlocks) {
    //     pistonControl(2, true);
    //     delay(600);
    //     pistonControl(2, false); 
    //       pixy.setLamp(0, 0);
    //       break;
    //     }
    //   }
    //   delay(200);
    //   servomotion(90,15);
    //   rollers();
    //   delay(500);
    //   pistonControl(1, false);
    //   servo_stopper.write(0);  
    // }
    
    // WITHOUT Dribbling - RECEIVING to CUP via Piston 

    if(l1 && button_16_flag == false){ // 0x10 - Button 16
      // process start
      button_16_flag = true;  
      pistonControl(1, true);   
      servomotion(20,92);
      pistonControl(2, true);
      servo_stopper.write(90); 
      delay(1000);
      pistonControl(0, true);
      delay(50);
      rollers();
    }else if(l1 && button_16_flag == true){
      // process wrap
      button_16_flag = false;
      rollers();
      pistonControl(1, false);   
      servomotion(92,20);
      pistonControl(2, false);
      servo_stopper.write(0); 
      pistonControl(0, false);
    }




    // FEEDER 

    if(triangle && triangle != last_triangle){
      //feed up
      digitalWrite(feeder_dir , LOW);
      analogWrite(feeder_pwm , 255*48);
    }else if(cross && cross != last_cross){
      //feed down
      digitalWrite(feeder_dir , HIGH);
      analogWrite(feeder_pwm , 255*48);
    }else if(r1 && r1 != last_r1){
      //stopSer==
      
      digitalWrite(feeder_dir , LOW);
      analogWrite(feeder_pwm , 255*0);
    }

    // RECIEVING 

    Serial.printf("Square: %d , flag_r: %d , flag_l: %d   " , square , flag_rec_r , flag_rec_l);
    if (l2 && flag_rec_r == false && flag_rec_l == false){
      Serial.print("REC UP: ");
      limitSwitchState[3] = digitalRead(limitSwitch[3]);
      Serial.printf("Inside if: %d   \n" , square);
      limitSwitchState[5] = digitalRead(limitSwitch[5]);

      Serial.print(limitSwitchState[3]);
      Serial.print(limitSwitchState[5]);


      while (limitSwitchState[3] != LOW || limitSwitchState[5] != LOW){
        int v = limitSwitchState[3]?(255*64):0;
        digitalWrite(rcv_dir[0], limitSwitchState[3]);
        analogWrite(rcv_pwm[0], v);

        v = limitSwitchState[5]?(255*64):0;
        digitalWrite(rcv_dir[1], limitSwitchState[5]);
        analogWrite(rcv_pwm[1], v);
        
        limitSwitchState[3] = digitalRead(limitSwitch[3]);
        limitSwitchState[5] = digitalRead(limitSwitch[5]);

        flag_rec_r = !limitSwitchState[3];
        flag_rec_l = !limitSwitchState[5];
        Serial.printf("Inside Loop: %d   \n" , square);
      }

      // flag_rec_r = !limitSwitchState[3];
      // flag_rec_l = !limitSwitchState[5];

      Serial.print(limitSwitchState[3]);
      Serial.print(limitSwitchState[5]);
 
      // digitalWrite(rcv_dir[0], limitSwitchState[3]);
      analogWrite(rcv_pwm[0], 0);
 
      digitalWrite(rcv_dir[1], limitSwitchState[5]);
      analogWrite(rcv_pwm[1], 0);
      // counter = 1;
      Serial.printf("Inside if part two: %d   \n" , square);
      // square = 0;
    }
    else if(square && flag_rec_r == true && flag_rec_l == true){
      // counter = 0;
      Serial.print("REC DOWN:");

      limitSwitchState[2] = digitalRead(limitSwitch[2]);
      limitSwitchState[4] = digitalRead(limitSwitch[4]);

      Serial.print(limitSwitchState[2]);
      Serial.print(limitSwitchState[4]);

      while (limitSwitchState[2] != LOW || limitSwitchState[4] != LOW){

        int v = limitSwitchState[2]?(255*64):0;
        digitalWrite(rcv_dir[0], !limitSwitchState[2]);
        analogWrite(rcv_pwm[0], v);

        v = limitSwitchState[4]?(255*64):0; 
        digitalWrite(rcv_dir[1], !limitSwitchState[4]);
        analogWrite(rcv_pwm[1], v);

        limitSwitchState[2] = digitalRead(limitSwitch[2]);
        limitSwitchState[4] = digitalRead(limitSwitch[4]);

        flag_rec_r = limitSwitchState[2];
        flag_rec_l = limitSwitchState[4];
      }

      // flag_rec_r = limitSwitchState[2];
      //   flag_rec_l = limitSwitchState[4];

      digitalWrite(rcv_dir[0], limitSwitchState[2]);
      analogWrite(rcv_pwm[0], 0);

      digitalWrite(rcv_dir[1], limitSwitchState[4]);
      analogWrite(rcv_pwm[1], 0);

      Serial.print(limitSwitchState[2]);
      Serial.print(limitSwitchState[4]);
    }
for(int i=0;i<6;i++){
  limitSwitchState[i]=digitalRead(limitSwitch[i]);
Serial.print(limitSwitchState[i]);
}
Serial.println();

last_haty = haty;
last_r2 = r2;
last_triangle = triangle;
last_cross = cross;
last_r1 = r1;
last_square = square;

lx=0; ly=0; rx=0; ry=0;l1=0; r1=0; circle=0; triangle=0; cross=0; square=0;hatx=0;haty=0;l2=0;r2=0;


    // SHOOTING

    // if(nrpm != 0 && (buttons & 0x04 && button_4_flag == false)){
    //   button_4_flag = true;
    //   while(orpm < nrpm){
    //     orpm=orpm+((nrpm-orpm)/700)+((nrpm-orpm)%700);
    //     UART.setRPM(orpm);
    //     delay(50);
    //     // Serial.println("+");
    //     // Serial.println((orpm/7));
    //   }
    //   while(orpm > nrpm){
    //     orpm=orpm-((orpm-nrpm)/700)-((orpm-nrpm)%700);
    //     UART.setRPM(orpm);
    //     delay(50);
    //     // Serial.println("-");
    //     // Serial.println((orpm/7));
    //   }        
    // }else if(nrpm == 0 || (buttons & 0x04 && button_4_flag == true)){
    //   button_4_flag = false;
    //   nrpm = 0;
    //   orpm = 0; 
    //   UART.setRPM(orpm);
    // } 
    // UART.setRPM(orpm); 
      }
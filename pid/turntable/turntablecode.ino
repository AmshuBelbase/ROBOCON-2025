#include <QNEthernet.h>

using namespace qindesign::network;

EthernetUDP udp;
IPAddress myIP(192, 168, 1, 101); // Match your Jetson's UDP_IP
uint16_t port = 12345;  


int direc_pin=9;
int pwm_pin=8;
int turn_pwm=0;
int maxpwm=230;
int count_not_detected = 0;

void setup() {
  
  pinMode(direc_pin,OUTPUT);
  pinMode(pwm_pin,OUTPUT);
  Serial.begin(115200);
  Ethernet.begin();
  Ethernet.setLocalIP(myIP);
  udp.begin(port);
  
  Serial.print("UDP Receiver ready at IP: ");
  Serial.println(Ethernet.localIP());

}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char packet[64];
    int len = udp.read(packet, sizeof(packet) - 1); // Read packet
    if (len > 0) {
      packet[len] = '\0'; // Null-terminate the packet
      int parsed = sscanf(packet, "%d", &turn_pwm); // Parse the integer from the packet
      if (parsed == 1) {
        Serial.printf("Received X-Offset: %d\n", turn_pwm); // Print received value
      } else {
        Serial.println("Packet Parsing Error");
      }
    }
    udp.flush(); 
  }

  //pwm control but linear map horaha 
  // int pwm =map(abs(x_offset),0,960,0,maxpwm);
  //   pwm=constrain(pwm,0,maxpwm);
  //  exponential mapping if leniar does not work
  // float exp_pwm = pow(abs(x_offset), 0.8);
  //    exp_pwm = map(exp_pwm, 0, pow(960, 0.8), 0, maxPWM);
      
  //      pwm = (int)exp_pwm;

  //direction control 

  // check if hoop is not detected for continuos 5 times (frames), if not detected, stop the motor.
  if(turn_pwm==0){
    count_not_detected++;
    if(count_not_detected == 5){
      count_not_detected = 0;
      digitalWrite(direc_pin,LOW);
      analogWrite(pwm_pin,0);
    }
  }
  // 1 represents the hoop is in center
  else if(turn_pwm == 1){
    count_not_detected = 0;
    digitalWrite(direc_pin,LOW);
    analogWrite(pwm_pin,0);
  }
  // if not in center rotate accordingly
  else if(turn_pwm!=0){
    count_not_detected = 0;
    digitalWrite(direc_pin,(turn_pwm>0)?LOW:HIGH);
    analogWrite(pwm_pin,abs(turn_pwm));
  }
  // stop at any extreme (undetermined) conditions
  else{
    digitalWrite(direc_pin,LOW);
    analogWrite(pwm_pin,0);
  }
}
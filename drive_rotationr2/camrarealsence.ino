// Move ClientServerEthernet.h to a subfolder of your Arduino/libraries/ directory.
#include <ClientServerEthernet.h>

// <<<< IMPORTANT ----

// Ensure the struct is packed with no padding between members.
// This is important for consistent memory layout, especially when sending data over serial or network.
// #pragma pack(1) → No padding (tightest packing).

// int a = 1000; -> size may vary (typically 4 bytes) across systems and compilers
// int16_t b = 1000; -> guaranteed to be 2 bytes across all platforms

// ---- IMPORTANT >>>>

#pragma pack(push, 1) // save current alignment and set to 1 byte
struct ControllerData { 
    int32_t axis[4]; 
    int32_t l2;
    int32_t r2;
    int16_t r1;
    int16_t l1;
    int16_t cross;
    int16_t square;
    int16_t circle;
    int16_t triangle; 
    int16_t turn_pwm;
    int16_t bldc_pwm;
};
#pragma pack(pop) // restore previous alignment


ControllerData jetdata; // Struct instance to hold incoming controller data

ClientServerEthernet<ControllerData> con; // Instance of the ClientServerEthernet class templated with ControllerData



//m1
int dir1_pin=9;
int pwm1_pin=2;
//m2
int dir2_pin=8;
int pwm2_pin=6;
// m3
int dir3_pin=4;
int pwm3_pin=3;
//  python side se aega ye 
int offset_pwm=0;
int count_not_detected = 0;

void setup() {
  pinMode(dir1_pin,OUTPUT);
  pinMode(pwm1_pin,OUTPUT);

  pinMode(dir2_pin,OUTPUT);
  pinMode(pwm2_pin,OUTPUT);

  pinMode(dir3_pin,OUTPUT);
  pinMode(pwm3_pin,OUTPUT);
  Serial.begin(115200);

  Serial.begin(115200);
  
  vector<int> client_ip = {192, 168, 1, 101}; // IP address of this device (client)
  vector<int> server_ip = {192, 168, 1, 102}; // IP address of the server to communicate with
  vector<int> subnet_mask = {255, 255, 255, 0}; // Subnet mask for the network

  // Initialize the Ethernet client-server connection with IPs, subnet, and a pointer to the data structure
  con = ClientServerEthernet<ControllerData>(client_ip, subnet_mask, server_ip, &jetdata);
}



void loop() {
    con.getData();
  Serial.println(jetdata.turn_pwm);
  delay(3);
  if(offset_pwm==0){
    count_not_detected++;
    if(count_not_detected == 5){
      count_not_detected = 0;
      digitalWrite(dir1_pin,LOW);
      analogWrite(pwm1_pin,0);
      digitalWrite(dir2_pin,LOW);
      analogWrite(pwm2_pin,0);
      digitalWrite(dir3_pin,LOW);
      analogWrite(pwm3_pin,0);
    }
  }
  // 1 represents the hoop is in center
  else if(offset_pwm == 1){
    count_not_detected = 0;
      digitalWrite(dir1_pin,LOW);
      analogWrite(pwm1_pin,0);
      digitalWrite(dir2_pin,LOW);
      analogWrite(pwm2_pin,0);
      digitalWrite(dir3_pin,LOW);
      analogWrite(pwm3_pin,0);
  }
  // if not in center rotate accordingly
  else if(offset_pwm!=0){
    count_not_detected = 0;
    digitalWrite(dir1_pin,(offset_pwm>0)?LOW:HIGH);
    analogWrite(pwm1_pin,abs(offset_pwm));
     digitalWrite(dir2_pin,(offset_pwm>0)?LOW:HIGH);
    analogWrite(pwm2_pin,abs(offset_pwm));
     digitalWrite(dir3_pin,(offset_pwm>0)?LOW:HIGH);
    analogWrite(pwm3_pin,abs(offset_pwm));
  }
  // stop at any undetermined conditions
  else{
    digitalWrite(dir1_pin,LOW);
    analogWrite(pwm1_pin,0);
    digitalWrite(dir2_pin,LOW);
    analogWrite(pwm2_pin,0);
    digitalWrite(dir3_pin,LOW);
    analogWrite(pwm3_pin,0);
  }
}

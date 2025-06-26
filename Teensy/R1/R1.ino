// Move ClientServerEthernet.h to a subfolder of your Arduino/libraries/ directory.
#include <ClientServerEthernet.h>
#include <Encoder.h>
#include <VescUart.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

float easeOutExpo(float x) {
  if (x == 1.0) {
    return 1.0;
  } 
  else {
    return 1.0 - pow(2.0, -10.0 * x);
  }
}

class PCAHandler {
  public:
    Adafruit_PWMServoDriver pwm;
    uint8_t i2c_address;
    bool initialized = false;

    PCAHandler(uint8_t address) : pwm(address), i2c_address(address), initialized(false) {}

    void tryInitialize() {
      Wire.beginTransmission(i2c_address);
      if (Wire.endTransmission() == 0) {
        if(!initialized) {
          pwm.begin();
          pwm.setPWMFreq(50); // For relays and servos
        }
        initialized = true; 
      } 
      else {
        if(initialized) {}
        initialized = false;
      }
    }
};

PCAHandler pca(0x41); // Change to 0x40 if that's your PCA address

IntervalTimer targetLocking;

bool flag_bldc=false;
bool data_update=true;

VescUart UART;
IntervalTimer pidTimer;

int pwmL_pin[3] = { 2, 0, 6 };
int pwmR_pin[3] = { 3, 1, 7 };

int drib_pwmL=12;
int drib_pwmR=13;

int feed_pwmL=5;
int feed_pwmR=4;

int drib_in=3;
int drib_out=2;

int target_pwmL=22;
int target_pwmR=23;

int bldc_rpm=0;
Encoder m[3] = { Encoder(21,20), Encoder(26,27), Encoder(41,40) };

volatile float rpm_rt[3] = { 0, 0, 0 };


int duty_cycle = 100;                           //in percentage
// int max_pwm = (int)(duty_cycle / 100.0 * res);  //6v--250rpm
int max_rpm = 300;

// int ii=0;
// <<<< IMPORTANT ----

// Ensure the struct is packed with no padding between members.
// This is important for consistent memory layout, especially when sending data over serial or network.
// #pragma pack(1) → No padding (tightest packing).

// int a = 1000; -> size may vary (typically 4 bytes) across systems and compilers
// int16_t b = 1000; -> guaranteed to be 2 bytes across all platforms

// ---- IMPORTANT >>>>

#pragma pack(push, 1) // save current alignment and set to 1 byte - 6i9h
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
  int16_t touch_button;
  int16_t turn_pwm;
  int16_t bldc_rpm;
};
#pragma pack(pop) // restore previous alignment

ControllerData jetdata; // Struct instance to hold incoming controller data
ClientServerEthernet<ControllerData> con; // Instance of the ClientServerEthernet class templated with ControllerData
uint32_t checkTimer = millis();

double mapDouble(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void target_locking()
{
  if(jetdata.touch_button) {
    if(abs(jetdata.turn_pwm)<200){ 
      runMotor(-jetdata.turn_pwm*64,target_pwmL,target_pwmR);
    } 
  }
  else
    runMotor(0*64,target_pwmL,target_pwmR);
}

void setup() {
  Serial.begin(115200);

  // pinMode(13,OUTPUT);
  // digitalWrite(13,HIGH);

  // pinMode(drib_dir,OUTPUT);
  // // pinMode(feed_pwmR,OUTPUT);
  // pinMode(rot_dir,OUTPUT);


  //  for (int i = 0; i < 3; i++) 
  // {
  //   // analogWriteFrequency(pwmL_pin[i], 9000);
  //   // pinMode(pwmR_pin[i], OUTPUT);
  // }

  Wire.begin();
  delay(1000);
  // Retry loop until PCA9685 is available
  while (!pca.initialized) {
    pca.tryInitialize();
    delay(500); // Wait before retrying
  }
  // pca.pwm.setPWM(drib_in,0,0);
  // pca.pwm.setPWM(drib_out,0,0);
  // delay(500);
  // pca.pwm.setPWM(drib_in,0,0);
  // pca.pwm.setPWM(drib_out,0,0);
  analogWriteResolution(14);
  
  vector<int> client_ip = {192, 168, 1, 101}; // IP address of this device (client)
  vector<int> server_ip = {192, 168, 1, 100}; // IP address of the server to communicate with
  vector<int> subnet_mask = {255, 255, 255, 0}; // Subnet mask for the network

  // Initialize the Ethernet client-server connection with IPs, subnet, and a pointer to the data structure
  con = ClientServerEthernet<ControllerData>(client_ip, subnet_mask, server_ip, &jetdata);
  pidTimer.begin(pid, 75000);
  targetLocking.begin(target_locking,50000);
  Serial8.begin(115200);

  // while (!Serial8) { ; }
  /** Define which ports to use as UART */
  UART.setSerialPort(&Serial8);
}

volatile long oldPosition[3] = { 0, 0, 0 };
int ledState = LOW;
volatile long count[3] = { 0, 0, 0 };  // use volatile for shared variables
volatile long newPosition[3] = { 0, 0, 0 };

volatile int pwm_pid[] = { 0, 0, 0 };
volatile float rpm_sp[] = { 0, 0, 0 };


volatile float kp[] = { 09.0, 09.0, 09.0 };
volatile float ki[] = { 165.0, 165.0, 165.0 };
volatile float kd[] = { 00.50, 00.50, 00.50 };

float error[] = { 0, 0, 0 };
float eInt[] = { 0, 0, 0 };
float eDer[] = { 0, 0, 0 };
float lastError[] = { 0, 0, 0 };
    
int y=0;
int x=0;
int w = 0;

void pid() {
    // ii++;
    // con.getData(true);
  for (int i = 0; i < 3; i++) {
    newPosition[i] = m[i].read();
    ::count[i] = abs(newPosition[i] - oldPosition[i]);
    // count=newPosition<oldPosition?-count:count;
    rpm_rt[i] = ::count[i] / 1300.0 * 600 * 4 / 3;
    rpm_rt[i] *= newPosition[i] < oldPosition[i] ? -1 : 1;
    // Serial.printf("RPM_output(motor: %d):%0.2f ", i + 1, rpm_rt[i]);
    ::count[i] = 0;
    oldPosition[i] = newPosition[i];
  }
  // if(ii%10==0)
  // Serial.printf("\n");
  if(data_update){
    int psAxisX = 0;
    int psAxisY = 0;
    if (jetdata.axis[0] < 120)
      psAxisX = map(jetdata.axis[0], 120, 0, 0, -255);\
    else if (jetdata.axis[0] > 135)
      psAxisX = map(jetdata.axis[0], 135, 255, 0, 255);
    else
      psAxisX = 0;
          
    if (jetdata.axis[1] > 135)
      psAxisY = map(jetdata.axis[1], 135, 255, 0, -255);
    else if (jetdata.axis[1] < 120)
      psAxisY = map(jetdata.axis[1], 120, 0, 0, 255);
    else
      psAxisY = 0;

    if (jetdata.axis[2] > 135)
      w = map(jetdata.axis[2], 135, 255, 0, 255);
    else if (jetdata.axis[2] < 120)
      w = map(jetdata.axis[2], 120, 0, 0, -255);
    else
      w=0;

    // else
    // if(jetdata.r2)
    //   w = jetdata.r2;
    // else
    //   w = -1*jetdata.l2;

    y = -psAxisY;
    x = -psAxisX;

    // Serial.print(x);
    // Serial.print("   ok ");
    // Serial.print(y);
    // Serial.println();
    // x=0;

    Serial.print("Rotation:");
    Serial.println(w);
  }

  rpm_sp[0] = map(x + 0.1*w, -175, 175, max_rpm, -max_rpm);
  rpm_sp[1] = map(-0.5 * x - 0.852 * y + 0.1*w, -175, 175, max_rpm, -max_rpm);
  rpm_sp[2] = map(-0.5 * x + 0.866 * y + 0.1*w, -175, 175, max_rpm, -max_rpm);

  // for (int i = 0; i < 3; i++) {
  //  Serial.printf("RPM_%d_input:%0.2f  ", i + 1, rpm_sp[i]);
  // }

  //~~this block of code is to take the input from the ps4 controller
  for (int i = 0; i < 3; i++) {
    error[i] = rpm_sp[i] - rpm_rt[i];
    eDer[i] = (error[i] - lastError[i]) / 0.075;
    eInt[i] = eInt[i] + error[i] * 0.075;

    pwm_pid[i] = int(kp[i] * error[i] + ki[i] * eInt[i] + kd[i] * eDer[i]);
    //Serial.printf("pwm_pid:%d ",pwm_pid[i]);
    // pwm_pid[i]=map(pwm_pid[i],-16383,16383,-pwm_18,pwm_18);
    //Serial.printf("pwm_pid:%d \n",pwm_pid[i]);
    pwm_pid[i]=pwm_pid[i]%16383;
    analogWrite(pwmR_pin[i], pwm_pid[i]>=0?pwm_pid[i]:0);
    analogWrite(pwmL_pin[i], pwm_pid[i]<=0?pwm_pid[i]*-1:0);
    lastError[i] = error[i];
    // Serial.printf("RPM_%d_input:%0.2f  ",i+1, rpm_sp[i]);
  }

  if(flag_bldc) {
    UART.setRPM(jetdata.bldc_rpm*7);
  }
  //else{
  //  UART.setRPM(0);
  //}
}

void runMotor(int pwm_val, int pwmLPin, int pwmRPin) {
  analogWrite(pwmLPin, (pwm_val <= 0 ? pwm_val*-1 : 0));
  analogWrite(pwmRPin, (pwm_val >= 0 ? pwm_val : 0));
}

bool shoot_flag=false;
// int lastTime=0;

void delayCustom(int n) {
  int lastTime=millis();
  while(millis()-lastTime<n) {
    con.getData();
  }
}

void loop() {
  if (millis() - checkTimer > 1000) {
    pca.tryInitialize();
    checkTimer = millis();
  }

  Serial.print("Turn PWM: ");
  Serial.println(jetdata.turn_pwm);

  con.MaintainConnection(false);
  con.getData();
  // Serial.printf("%d %d %d %d %d %d %d %d %d %d %d %d %d ",);

  if(jetdata.circle==1) {
    // digitalWrite(rot_dir,LOW);
    // analogWrite(rot_pwm,37*64);
    // delay(1400);
    // analogWrite(rot_pwm,0);
    // delay(800);
    // digitalWrite(drib_dir,LOW);
    // analogWrite(drib_pwm,255*64);
    // delay(600);
    // analogWrite(drib_pwm,0);
    Serial.println("Dribbling: circle");
    pca.pwm.setPWM(drib_in,0,0);
    pca.pwm.setPWM(drib_out,0,4095);
    delay(1000);
    pca.pwm.setPWM(drib_in,0,0);
    pca.pwm.setPWM(drib_out,0,0);
    delay(200);
    runMotor(255*64,drib_pwmL,drib_pwmR);
    delay(300);
    runMotor(-255*64,drib_pwmL,drib_pwmR);
    delay(1000);
    runMotor(0*64,drib_pwmL,drib_pwmR);
    delay(200);
    pca.pwm.setPWM(drib_in,0,4095);
    pca.pwm.setPWM(drib_out,0,0);
    delay(1000);
    pca.pwm.setPWM(drib_in,0,0);
    pca.pwm.setPWM(drib_out,0,0);
  }

  if(jetdata.cross==1) {
    // runMotor(-255*64,feed_pwmL,feed_pwmR);
    runMotor(255*64,drib_pwmL,drib_pwmR);
    delay(300);
    // runMotor(0*64,feed_pwmL,feed_pwmR);
    runMotor(-255*64,drib_pwmL,drib_pwmR);
    delay(75);
    runMotor(0*64,drib_pwmL,drib_pwmR);
      
  }
  if(jetdata.square==1) {
    runMotor(0*64,feed_pwmL,feed_pwmR);
    // runMotor(255*64,drib_pwmL,drib_pwmR);
    // delay(1500);
    // runMotor(0*64,feed_pwmL,feed_pwmR);
    // runMotor(0*64,drib_pwmL,drib_pwmR);
  }

  if(jetdata.r1) {
    flag_bldc=true;
    runMotor(0,drib_pwmL,drib_pwmR);
    delay(1000);
    runMotor(255*64,feed_pwmL,feed_pwmR);
    // delay(3000);
    // lastTime=millis();
    // while(millis()-lastTime<3000){
    // con.getData();
    // // Serial.println("hjngbfvdcjkghjgjhfvghcghcfgxcjhg");
    // }

    delay(3000);
    flag_bldc=false;
    // runMotor(0*64,drib_pwmL,drib_pwmR);
    runMotor(0*64,feed_pwmL,feed_pwmR);    
  }
  
  if(jetdata.r2>10) {
    pca.pwm.setPWM(drib_out,0,4095);
    pca.pwm.setPWM(drib_in,0,0);
    delay(1000);
    pca.pwm.setPWM(drib_in,0,0);
    pca.pwm.setPWM(drib_out,0,0);
  }
  
  if(jetdata.l2>10) {
    pca.pwm.setPWM(drib_in,0,4095);
    pca.pwm.setPWM(drib_out,0,0);
    delay(1000);
    pca.pwm.setPWM(drib_in,0,0);
    pca.pwm.setPWM(drib_out,0,0);
  }
  
  // if(jetdata.triangle==1) {
  //   runMotor(255*64,drib_pwmL,drib_pwmR);
  //   delay(200);
  //   runMotor(-127*64,drib_pwmL,drib_pwmR);
  //   delay(300);


  //   delay(200);
  //   digitalWrite(feed_pwmR,LOW);
  //   analogWrite(feed_pwmL,255*64);
  //   drive(-1*255*64,feed_pwmL,feed_pwmR);
  //   delay(600);
  //   digitalWrite(drib_dir,HIGH);
  //   analogWrite(drib_pwm,255*64);
  //   delay(600);
  //   digitalWrite(drib_dir,LOW);
  //   analogWrite(drib_pwm,255*64);
  //   delay(1200);
  //   flag_bldc=false;
  //   data_update=false;
  //   y=-65;
  //   delay(750);
  //   y=0;
  //   data_update=true;
  //   digitalWrite(drib_dir,LOW);
  //   analogWrite(drib_pwm,0*64);    
  // }

  delay(10);
}
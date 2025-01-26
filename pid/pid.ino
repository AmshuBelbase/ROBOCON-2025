#include <Encoder.h>
#include <IntervalTimer.h> 
#include "USBHost_t36.h"

Encoder myEnc(9, 8);

//PS4 connection 
USBHost myusb;   // initializes and manages the USB host port, enabling Teensy to detect and commuincate with USB bluetooth dongle
USBHIDParser hid1(myusb);  //works behind the scenes to parse HID data that comes from the PS4 controller, such as joystick movements and button presses.
JoystickController joystick(myusb);
BluetoothController bluet(myusb, true, "0000");   // Version does pairing to device

//coordinates of joystick (x,y -> right joystick; leftX -> left joystick)
int y; 

// motor pins
int PWM = 1;      
int DIR = 0;

volatile long encoderCounts = 0;  // Shared variable to track encoder counts
volatile long lastCount = 0;      
volatile double rpm = 0;          // Stores the calculated RPM
long positionChange;

//pid constants
float kp = 0.0;    // 20
float ki = 0.0;    // 80
float kd = 0.0;    // 0.5

float sp=0.0;
float pid=0.0;
float prev_err=0.0;
float integ=0.0;
float der=0.0;

IntervalTimer timer; // Timer object for periodic execution

void calculatePID() {
  unsigned long startTime = micros();

  if (Serial.available() > 0) {
  
  String input = Serial.readString();

  kp = input.substring(0,3).toFloat();
  ki = input.substring(3,6).toFloat();
  kd = input.substring(6).toFloat();
  }

  myusb.Task();   // Handle USB host tasks

  if (joystick.available()) {
    int rightStickY = joystick.getAxis(1);
    y = map(rightStickY, 0, 255, 127, -127);
    
    // Ignore small joystick values
    if (abs(y) < 5) y = 0;
   
    sp = map(y, -127, 127, -250, 250);  //mapping joystick to rpm range
  } else {
    Serial.println("Joystick Not Found");
  }

  long currentCounts = myEnc.read();
  positionChange = currentCounts - lastCount;
  lastCount = currentCounts;

  // Calculate RPM
  rpm = (positionChange / 1300.0) * (60 * (1000.0 / 75));
  //Serial.print("rpm: ");
  //Serial.println(rpm);


  //PID Control
  float err = sp - rpm;
  //Serial.print("error: ");
  //Serial.println(err);
  integ = integ + (err*0.075);             //integ += (err-prev_err);
  der = (err-prev_err)/0.075;
  //integ =constrain(integ,-100,100);
  pid = (kp*err) + (ki*integ) + (kd*der);
  prev_err = err;

  pid = constrain(pid, -16383, 16383);
  //Serial.print("y: ");
  //Serial.println(y);

  int pwmValue = abs(pid); 
  if (pid > 0) {      //to check direction: if +ve - HIGH, else LOW
    digitalWrite(DIR, HIGH);
  } else if (pid < 0) {
    digitalWrite(DIR, LOW);
  } else {
    pwmValue = 0;
  }
  analogWrite(PWM, pwmValue);

  Serial.print("sp:");
  Serial.print(sp);
  Serial.print(" rpm:");
  Serial.println(rpm);

  unsigned long currentTime = micros();
  unsigned long time = currentTime-startTime;
  // Serial.println(time);
  
}

void setup() {
  Serial.begin(9600);

  // Motor control pins setup
  pinMode(PWM, OUTPUT);
  pinMode(DIR, OUTPUT);

  // Initialize motor to stop
  analogWrite(PWM, 0);
  digitalWrite(DIR, LOW);

  analogWriteResolution(14);
  analogWriteFrequency(0, 9000);

  myusb.begin();
  delay(2000);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  timer.begin(calculatePID, 75000);
}

void loop() {

}
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Pixy2.h>

// Create PCA9685 and Pixy2 objects
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
Pixy2 pixy;

// PCA9685 channels
#define rc0    0   // Relay 0
#define rc1    1   // Relay 1
#define rc2    2   // Relay 2
#define rc3    3   // Relay 3
#define servo0 7   // Servo 0
#define servo1 5   // Servo 1
#define stepCh 13   // Stepper Step channel
#define dirCh  14  // Stepper Direction channel

// Relay states
bool rs0 = false;
bool rs1 = false;
bool rs2 = false;
bool rs3 = false;

// Stepper motor state
bool stepperRunning = false;
bool stepperDir = true;

void setup() 
{
  Serial.begin(9600);
  Serial.println("Relay, Servo, Stepper, and Pixy2 Detection");

  // Initialize PCA9685
  pwm.begin();
  pwm.setPWMFreq(50);  // For servos and relays (50 Hz)

  // Initialize all outputs to LOW
  //pwm.setPWM(channel, ON, OFF)
  // ON tick in duty cycle 
  // OFF tick in duty cycle 
  pwm.setPWM(rc0, 0, 0);
  pwm.setPWM(rc1, 0, 0);
  pwm.setPWM(rc2, 0, 0);
  pwm.setPWM(rc3, 0, 0);
  pwm.setPWM(stepCh, 0, 0);
  pwm.setPWM(dirCh, 0, 0);

  // Initialize Pixy2
  pixy.init();
}

void loop() 
{
  /*
  // Pixy Detection
  pixy.setLamp(1, 1);      // Lamp ON for better detection
  pixy.ccc.getBlocks();    // Check for objects

  if (pixy.ccc.numBlocks) {
    Serial.println("Detected by Pixy!");
    pixy.setLamp(0, 0);    // Lamp OFF after detection
  } else {
    Serial.println("Not Detected by Pixy");
  }
  
*/
  // Serial input to control devices
  while (Serial.available()) 
  {
    char ch = Serial.read();

    // Relay controls
    if (ch == '0') toggleRelay(rc0, rs0);
    if (ch == '1') toggleRelay(rc1, rs1);
    if (ch == '2') toggleRelay(rc2, rs2);
    if (ch == '3') toggleRelay(rc3, rs3);

    // Servo sweep control
    if (ch == 'u') {
      sweepBothServos();
      Serial.println("Servos Sweeping");
    }

    // Stepper motor controls
    if (ch == 'w') {
      pwm.setPWM(dirCh, 0, 4095);  // Forward
      stepperDir = true;
      startStepper();
      Serial.println("Stepper: Forward");
    }

    if (ch == 's') {
      pwm.setPWM(dirCh, 0, 0);     // Backward
      stepperDir = false;
      startStepper();
      Serial.println("Stepper: Backward");
    }

    if (ch == 'x') {
      stopStepper();
      Serial.println("Stepper: Stopped");
    }
  }
}

// Function to toggle relay ON/OFF
void toggleRelay(uint8_t channel, bool &state) 
{
  state = !state; // Toggle state
  pwm.setPWM(channel, 0, state ? 4095 : 0);
  Serial.print("Relay ");
  Serial.print(channel);
  Serial.print(": ");
  Serial.println(state ? "ON" : "OFF");
}

// Function to set Servo Angle
void setServoAngle(uint8_t channel, int angle) 
{
  int pulse = map(angle, 0, 180, 150, 600);
  pwm.setPWM(channel, 0, pulse);
}

// Function to sweep both servos together
void sweepBothServos() 
{
  for (int angle = 0; angle <= 180; angle += 5) 
  {
    setServoAngle(servo0, angle);
    setServoAngle(servo1, angle);
    delay(20);
  }
  for (int angle = 180; angle >= 0; angle -= 5) 
  {
    setServoAngle(servo0, angle);
    setServoAngle(servo1, angle);
    delay(20);
  }
}

// Function to start Stepper Motor
void startStepper() {
  pwm.setPWMFreq(500);                  // Set high frequency for stepper (around 500Hz)
  pwm.setPWM(stepCh, 0, 2048);           // 50% duty cycle pulses
  stepperRunning = true;
}

// Function to stop Stepper Motor
void stopStepper() {
  pwm.setPWM(stepCh, 0, 0);              // No pulses
  pwm.setPWMFreq(50);                    // Reset frequency back for servo/relay
  stepperRunning = false;
}

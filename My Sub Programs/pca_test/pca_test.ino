#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Pixy2.h>

// Create PCA9685 and Pixy2 objects
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

#define rc0    7   // Relay 0
#define rc1    8   // Relay 1

// Relay states
bool rs0 = false;
bool rs1 = false;

void setup() 
{
  // Initialize PCA9685
  pwm.begin();
  pwm.setPWMFreq(50);  // For servos and relays (50 Hz)

  // Initialize all outputs to LOW
  //pwm.setPWM(channel, ON, OFF)
  // ON tick in duty cycle 
  // OFF tick in duty cycle 
  pwm.setPWM(rc0, 0, 0);
  pwm.setPWM(rc1, 0, 0);
}

void loop()
{
  // Serial input to control devices
  while (Serial.available()) 
  {
    char ch = Serial.read();

    // Relay controls
    if (ch == '0') toggleRelay(rc0, rs0);
    if (ch == '1') toggleRelay(rc1, rs1);
  }
}

void toggleRelay(uint8_t channel, bool &state) 
{
  state = !state; // Toggle state
  pwm.setPWM(channel, state ? 0 : 4095, state ? 4095 : 0);
  Serial.print("Relay ");
  Serial.print(channel);
  Serial.print(": ");
  Serial.println(state ? "ON" : "OFF");
}
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Pixy2I2C.h>

// Create Pixy2 and PCA9685 objects
Pixy2I2C pixy;
Adafruit_PWMServoDriver pwm;

// Channels for devices connected to PCA9685
//const int ledChannel    = 14;  // LED on channel 0
const int relayChannel  = 4;  // Relay on channel 4
const int servoChannel  = 0; // Servo on channel 15


// LED brightness levels (0 - 4095 for 12-bit PWM)
#define LED_LOW     500
#define LED_MEDIUM  2000
#define LED_HIGH    4095

// Servo pulse width range (depends on servo)
#define SERVO_MIN   500     // Minimum pulse for 0°
#define SERVO_MAX   2400    // Maximum pulse for 180°
#define SERVO_STEP  10      // Step in degrees for Q/W keys

bool relayState = false;      // Track relay ON/OFF state
int servoPosition = 90;       // Servo starts at 90° (middle)

// Setup function runs once
void setup() {
  Serial.begin(9600);
  Serial.println("Pixy2 + PCA9685: LED, Relay, Servo Control");

  // Initialize PCA9685
  pwm.begin();
  pwm.setPWMFreq(50);  // 50Hz for servo control

  // Set initial states
  //pwm.setPWM(ledChannel, 0, 0); // LED OFF
  pwm.setPWM(relayChannel, 0, 0); // Relay OFF
  pwm.setPWM(servoChannel, 0, map(servoPosition, 0, 180, SERVO_MIN, SERVO_MAX)); // Center servo

  // Initialize Pixy2
  pixy.init();
}

// Loop runs repeatedly
void loop() {
  // Pixy2 LED ON for visibility
  pixy.setLamp(1, 1);

  // Get blocks (object detection)
  pixy.ccc.getBlocks();

  // If any blocks are detected
  if (pixy.ccc.numBlocks) {
    Serial.println("Detected");
    pixy.setLamp(0, 0); // Optional: turn Pixy lamp off after detection
  } else {
    Serial.println("Not Detected");
  }

  // Handle serial input for controls
  handleSerialInput();

  delay(200); // Delay to prevent flooding serial monitor
}

// Serial control function
void handleSerialInput() {
  while (Serial.available()) {
    char ch = Serial.read();

/*
    // LED Brightness Controls
    if (ch == 'a' || ch == 'A') {
      pwm.setPWM(ledChannel, 0, LED_LOW);
      Serial.println("LED: LOW");
    }
    else if (ch == 's' || ch == 'S') {
      pwm.setPWM(ledChannel, 0, LED_MEDIUM);
      Serial.println("LED: MEDIUM");
    }
    else if (ch == 'd' || ch == 'D') {
      pwm.setPWM(ledChannel, 0, LED_HIGH);
      Serial.println("LED: HIGH");
    }
    else if (ch == 'x' || ch == 'X') {
      pwm.setPWM(ledChannel, 0, 0);
      Serial.println("LED: OFF");
    }
    */

    // Relay Toggle
    if (ch == 'r' || ch == 'R') {
      relayState = !relayState;
      pwm.setPWM(relayChannel, 0, relayState ? 4095 : 0);
      Serial.println(relayState ? "Relay: ON" : "Relay: OFF");
    }

    // Servo Control - Q to turn left, W to turn right
    else if (ch == 'q' || ch == 'Q') {
      servoPosition -= SERVO_STEP;
      if (servoPosition < 0) servoPosition = 0;
      pwm.setPWM(servoChannel, 0, map(servoPosition, 0, 180, SERVO_MIN, SERVO_MAX));
      Serial.print("Servo: ");
      Serial.print(servoPosition);
      Serial.println("° Left");
    }
    else if (ch == 'w' || ch == 'W') {
      servoPosition += SERVO_STEP;
      if (servoPosition > 180) servoPosition = 180;
      pwm.setPWM(servoChannel, 0, map(servoPosition, 0, 180, SERVO_MIN, SERVO_MAX));
      Serial.print("Servo: ");
      Serial.print(servoPosition);
      Serial.println("° Right");
    }
  }
}

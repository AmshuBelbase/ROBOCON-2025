#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Class to initialize PCA9685 with retry
class PCAHandler {
public:
  Adafruit_PWMServoDriver pwm;
  uint8_t i2c_address; // address = 0x40 or 0x41 based on pca 
  bool initialized = false;

  PCAHandler(uint8_t address) : pwm(address), i2c_address(address), initialized(false) {} //contrustor to initialize 

  void tryInitialize(bool relay_1_state, bool relay_2_state) {
    Wire.beginTransmission(i2c_address); //intilizing connection
    // works if pca found at set address
    if (Wire.endTransmission() == 0) {
      if(!initialized){
        pwm.begin();
        pwm.setPWMFreq(50); // For relays 
        pwm.setPWM(12, 0, relay_1_state?4095:0); // Channel 12 OFF
        pwm.setPWM(13, 0, relay_2_state?4095:0); // Channel 13 OFF
        Serial.print("PCA Intitalized with current state ✅");
      }
      initialized = true; 
    } else {
      if(initialized){
        Serial.print("PCA disconnected ❌");
      }
      initialized = false;
    }
  }
};

// Create PCA9685 handler object
PCAHandler pca(0x41);

// Relay channels
#define RELAY_1 12
#define RELAY_2 13

// Relay states
bool relay1_state = false;
bool relay2_state = false;

uint32_t checkTimer = millis();


void setup() {
  Serial.begin(9600);
  Wire.begin();
  delay(1000);
  // Retry loop until PCA9685 is available
  while (!pca.initialized) {
    pca.tryInitialize(relay1_state, relay2_state);
    delay(500); // Wait before retrying
  }

  Serial.println("Ready to receive commands: '1' for Relay 12, '2' for Relay 13.");
}

void loop() {
  if (millis() - checkTimer > 1000){
    pca.tryInitialize(relay1_state, relay2_state);
    checkTimer = millis();
  }
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '1') toggleRelay(RELAY_1, relay1_state);
    if (ch == '2') toggleRelay(RELAY_2, relay2_state);
  }
}

// Toggle relay ON/OFF
void toggleRelay(uint8_t channel, bool &state) {
  state = !state;
  pca.pwm.setPWM(channel, 0, state ? 4095 : 0);
  Serial.print("Relay ");
  Serial.print(channel);
  Serial.print(" is now ");
  Serial.println(state ? "ON" : "OFF");
}

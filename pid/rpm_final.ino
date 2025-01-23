#include <USBHost_t36.h>
#include <Encoder.h>

// USBHost and joystick setup
USBHost myusb;
USBHIDParser hid_parser(myusb);
JoystickController joystick(hid_parser);
BluetoothController bluet(myusb, true, "0000");
// Motor 1 control pins
const int motor1_pwm = 5;
const int motor1_dir = 4;

// Encoder for Motor 1
Encoder encoder1(12,11); // Motor 1 encoder

// Encoder and RPM calculation
const int CPR = 1300; // Counts per revolution of the encoder
unsigned long lastTimeRPM = 0;
long lastEncoderTicks = 0;
double actualRPM = 0;

// Desired RPM
double setpoint = 0; // Desired RPM (set by joystick)

// Helper function to set motor speed and direction
void setMotor(int pwmPin, int dirPin, double speed) {
  speed = constrain(speed, -16383, 16383); // Constrain PWM within range
  if (speed >= 0) {
    digitalWrite(dirPin, HIGH); // Forward direction
    analogWrite(pwmPin, speed);
  } else {
    digitalWrite(dirPin, LOW); // Reverse direction
    analogWrite(pwmPin, -speed);
  }
}

// Function to calculate RPM from encoder
double calculateRPM() {
  unsigned long now = millis();
  unsigned long timeChange = now - lastTimeRPM;

  if (timeChange >= 100) { // Update RPM every 100ms
    long encoderTicks = encoder1.read(); // Read current encoder ticks
    long tickChange = encoderTicks - lastEncoderTicks;

    // Convert tick change to RPM
    double revolutions = (double)tickChange / CPR; 
    actualRPM = (revolutions * 60000) / timeChange; // Calculate RPM from revolutions

    lastEncoderTicks = encoderTicks; // Update last tick count
    lastTimeRPM = now; // Update last time
  }
  return actualRPM; // Return the calculated RPM
}

void setup() {
  Serial.begin(9600);

  // Initialize motor and encoder
  pinMode(motor1_pwm, OUTPUT);
  pinMode(motor1_dir, OUTPUT);

  // Initialize USB Host for PS4 controller
  myusb.begin();
  
  analogWriteResolution(14); // Set resolution to 14-bit
  analogWriteFrequency(motor1_pwm, 20000); // Set PWM frequency to 20 kHz

}

void loop() {
  myusb.Task(); // Process USB tasks

  // Check if the joystick is available
  if (joystick.available()) {
    // Read joystick X-axis
    int joystickValue = joystick.getAxis(0); // Joystick value (0 to 255)

    // Map joystick input to RPM range (-200 to 200)
    setpoint = map(joystickValue, 0, 255, -200, 200);

    // Map joystick input to PWM range (-255 to 255)
    double pwmOutput = map(joystickValue, 0, 255, -5300, 5300);

    // Set motor speed and direction based on joystick input
    setMotor(motor1_pwm, motor1_dir, pwmOutput);
  } else {
    setpoint = 0; // Stop motor if joystick is not connected
    setMotor(motor1_pwm, motor1_dir, 0);
  }

  // Calculate actual RPM from encoder feedback
  actualRPM = calculateRPM();

  // Debugging information to monitor RPM
  Serial.print("Desired RPM (Setpoint): ");
  Serial.print(setpoint); // Desired RPM from joystick input
  Serial.print(", Actual RPM: ");
  Serial.println(actualRPM); // Measured RPM from encoder

  delay(100); // Small delay for stability
}

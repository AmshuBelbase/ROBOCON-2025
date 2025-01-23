#include <USBHost_t36.h>
#include <Encoder.h>
#include <IntervalTimer.h>

// USBHost and joystick setup
USBHost myusb;
USBHIDParser hid_parser(myusb);
JoystickController joystick(hid_parser);
BluetoothController bluet(myusb, true, "0000");

// Motor control pins
const int motor1_pwm = 19;
const int motor1_dir = 17;

// Encoder for Motor 1
Encoder encoder1(38, 39);

// Encoder and RPM calculation
const int CPR = 1300; // Counts per revolution
volatile double actualRPM = 0; 
volatile long lastEncoderTicks = 0; 
volatile double setpoint = 0; 

// PID variables
double kp = 0.5, ki = 0.0, kd = 0.0; 
volatile double errSum = 0, lastErr = 0;

// Timer setup
IntervalTimer motorTimer;

void setMotor(int pwmPin, int dirPin, double speed) {
  speed = constrain(speed, -16383, 16383); 
  if (speed >= 0) {
    digitalWrite(dirPin, HIGH); 
    analogWrite(pwmPin, speed);
  } else {
    digitalWrite(dirPin, LOW); 
    analogWrite(pwmPin, -speed);
  }
}

void timerISR() {
  

  // Encoder RPM calculation
  long encoderTicks = encoder1.read();
  long tickChange = encoderTicks - lastEncoderTicks;
  double revolutions = (double)tickChange / CPR;

  actualRPM = (revolutions * 60) / 0.1; 
  lastEncoderTicks = encoderTicks;

  // PID computation
  double error = setpoint - actualRPM;
  errSum = errSum + (error * 0.1); 
  double dErr = (error - lastErr) / 0.1; 

  double pidOutput = (kp * error) + (ki * errSum) + (kd * dErr);
  pidOutput = constrain(pidOutput, -16383, 16383); 

  lastErr = error;

  // Set motor speed
  setMotor(motor1_pwm, motor1_dir, pidOutput);
}


// Function to check and parse serial input for PID tuning
void checkSerialInput() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n'); // Read the full input line
    input.trim(); // Remove any whitespace or newlines

    // Find positions of commas
    int firstComma = input.indexOf(',');
    int secondComma = input.indexOf(',', firstComma + 1);

    // Check if input has the correct format
    if (firstComma > 0 && secondComma > firstComma) {
      // Extract substrings for kp, ki, and kd
      String kpStr = input.substring(0, firstComma);
      String kiStr = input.substring(firstComma + 1, secondComma);
      String kdStr = input.substring(secondComma + 1);

      // Convert to float and update PID variables
      kp = kpStr.toFloat();
      ki = kiStr.toFloat();
      kd = kdStr.toFloat();

      // Debugging: Print the updated values
      Serial.print("Updated kp: ");
      Serial.println(kp);
      Serial.print("Updated ki: ");
      Serial.println(ki);
      Serial.print("Updated kd: ");
      Serial.println(kd);
    } else {
      Serial.println("Invalid input. Format: kp,ki,kd (e.g., 43.4,0.01,0.01)");
  }
  }
}


void setup() {
  Serial.begin(9600);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  pinMode(motor1_pwm, OUTPUT);
  pinMode(motor1_dir, OUTPUT);

  analogWriteResolution(14);
  analogWriteFrequency(motor1_pwm, 20000);

  myusb.begin(); 
  motorTimer.begin(timerISR, 100000); 
}

void loop() {
  myusb.Task(); 

  if (joystick.available()) {
    int joystickValue = joystick.getAxis(0);
    setpoint = map(joystickValue, 0, 255, -200, 200); 
  } else {
    setpoint = 0;
  }

  // Debugging info
  noInterrupts(); 
  
  // Check for serial input for PID tuning
  checkSerialInput();
  

  double currentRPM = actualRPM;
  double currentSetpoint = setpoint;

  interrupts();

  //Serial.print("Setpoint: ");
  Serial.print(currentSetpoint);
  Serial.print ("  ");
  //Serial.print(" Actual RPM: ");
  Serial.println(currentRPM);

  delay(100); 
}
// TCS3200 Color Sensor and Motor Control with Potentiometer
// This code reads color values from a TCS3200 sensor and controls motor speed based on potentiometer input.

#include <tcs3200.h>  // Include the TCS3200 library for color sensing

// Variables to hold color readings
int red, green, blue, white;

// TCS3200 sensor configuration
/*
_____________________________
| Frequency Scaling | S0  | S1  |
|-------------------|-----|-----|
| 2%                | LOW | HIGH|
| 20%               | HIGH| LOW |
| 100%              | HIGH| HIGH|
-----------------------------
*/
tcs3200 tcs(9, 8, 11, 10, 12); // Pin configuration: (S0, S1, S2, S3, output pin) //constructor 

// Motor control pins
int IN1 = 4;    // IN1 controls motor direction
int IN2 = 7;    // IN2 controls motor direction
int ENA = 3;    // ENA for speed control using PWM

// Potentiometer configuration
int potPin = A1;   // Pin connected to the potentiometer for analog input
int potValue = 0;  // Variable to store the potentiometer value
int motorSpeed = 0; // Variable to hold motor speed based on potentiometer input

// Function to control the motor speed based on potentiometer input
void motor() {

  // Set motor direction to forward and control speed
  digitalWrite(IN1, LOW);  // Set motor direction (forward)
  digitalWrite(IN2, HIGH); // Set motor direction
  analogWrite(ENA, 255); // Adjust motor speed to highest using PWM

  // Output potentiometer and motor speed values to Serial monitor for debugging
  Serial.print("  Motor Speed (PWM): full");
  

  delay(500); // Short delay for stability in motor speed adjustments
}

void motorstopper(){
digitalWrite(IN1,LOW);
digitalWrite(IN2,LOW);
}

// Solenoid control pins
int solenoid1 = 6; // Pin for solenoid 1
int solenoid2 = 5; // Pin for solenoid 2

// Function to control solenoids for cylinder operation
void solenoid() {
  Serial.println("Activating Solenoid 1 (Extending cylinder)");
  digitalWrite(solenoid1, HIGH);  // Turn on solenoid 1 to extend cylinder
  digitalWrite(solenoid2, LOW);    // Ensure solenoid 2 is off
  delay(100);  // Keep solenoid 1 on for 100 milliseconds

  // Deactivate Solenoid 1 (stop cylinder movement)
  Serial.println("Deactivating Solenoid 1");
  digitalWrite(solenoid1, LOW);
  digitalWrite(solenoid2, LOW);
  delay(100);  // 1-second delay between operations

  // Activate Solenoid 2 (retract cylinder)
  Serial.println("Activating Solenoid 2 (Retracting cylinder)");
  digitalWrite(solenoid2, HIGH);  // Turn on solenoid 2 to retract cylinder
  digitalWrite(solenoid1, LOW);    // Ensure solenoid 1 is off
  delay(100);  // Keep it on for 100 milliseconds

  // Deactivate Solenoid 2 (stop cylinder movement)
  Serial.println("Deactivating Solenoid 2");
  digitalWrite(solenoid2, LOW);
  digitalWrite(solenoid1, LOW);
  delay(100);
}

// Arduino setup function
void setup() {
  pinMode(solenoid1, OUTPUT); // Set solenoid 1 pin as output
  pinMode(solenoid2, OUTPUT); // Set solenoid 2 pin as output
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(ENA,OUTPUT);
  Serial.begin(9600); // Initialize serial communication at 9600 bps
}

// Main loop function
void loop() {
   // Control motor speed based on potentiometer input

  // Read color values from TCS3200 sensor
  red = tcs.colorRead('r');     // Read red color value
  Serial.print("R= ");
  Serial.print(red);
  Serial.print("   ");
  
  green = tcs.colorRead('g');   // Read green color value
  Serial.print("G= ");
  Serial.print(green);
  Serial.print("   ");

  blue = tcs.colorRead('b');    // Read blue color value
  Serial.print("B=");
  Serial.print(blue);
  Serial.print("   ");

  white = tcs.colorRead('w');   // Read clear filter value
  Serial.print("W(clear)=");
  Serial.print(white);
  Serial.print("   ");
  
  Serial.println(); // Print a new line for better readability
  delay(200); // Delay for stability and reading

  // Determine actions based on color readings
  // Conditions to identify colors and activate solenoids
  if ((blue ==3) && (green == 2) && (red == 1)) {//FOR BLUE
    Serial.println("Detected: BLUE");
    motor();
    delay(1500); // Wait for a second
    solenoid();  // Activate solenoid
  } 
  else if ((blue == 2) && (green==2) && (red ==4||red==5)) {//FOR RED
    Serial.println("Detected: RED");
    motor();
    delay(2000); // Wait for two seconds
    solenoid();  // Activate solenoid
motorstopper();
  }
  else if ((blue == 4||blue==3) && (green == 5||green==4) && (red == 3||red==4)) {//FOR GREEN
    Serial.println("Detected: GREEN");
    motor();
    delay(2500); // Wait for three seconds
    solenoid();  // Activate solenoid
motorstopper();
  }
  else if ((blue == 2) && (green == 2) && (red==3)) {//FOR YELLOW
    Serial.println("Detected: YELLOW");
    motor();
    delay(3000); // Wait for four seconds
    solenoid();  // Activate solenoid
motorstopper();
  }
  if(white>=20 && red<=7){//FOR WHITE
    Serial.println("Detected: WHITE");
    motor();
   delay(5000);
motorstopper();
}
}
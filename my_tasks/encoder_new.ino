// Define pins for encoder channels A and B
 int encoderPinA = 2; // Encoder channel A to digital pin 2
 int encoderPinB = 3; // Encoder channel B to digital pin 3

long encoderPosition = 0; // Track the encoder's position
int lastEncoderStateA = LOW;       // Keep track of the last state of channel A
String rotationDirection = "";  

void setup() {
  
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);

  // turn pullup resistors on
  digitalWrite(encoderPinA, HIGH);
  digitalWrite(encoderPinB, HIGH);

  // Attach interrupt to encoder pin A (trigger on any change)
  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);

  Serial.begin(9600); // Begin serial communication for debugging
}

void loop() {
  // Print encoder position and direction
  Serial.print("Encoder Position: ");
  Serial.print(encoderPosition);
  Serial.print(" | Direction: ");
  Serial.println(rotationDirection);
  
  delay(100);
}

// Interrupt function to update the encoder position and direction
void updateEncoder() {
  int currentEncoderStateA = digitalRead(encoderPinA);
  int currentEncoderStateB = digitalRead(encoderPinB);

  // Determine the direction of rotation
  if (currentEncoderStateA != lastEncoderStateA) { // State change detected
    if (currentEncoderStateA == HIGH) {
      if (currentEncoderStateB == LOW) {
        encoderPosition++;                // Clockwise rotation
        rotationDirection = "Clockwise";
      } else {
        encoderPosition--;                // Counter-clockwise rotation
        rotationDirection = "anti-clockwise";
      }
    } else {
      if (currentEncoderStateB == HIGH) {
        encoderPosition++;                // Clockwise rotation
        rotationDirection = "Clockwise";
      } else {
        encoderPosition--;                // Counter-clockwise rotation
        rotationDirection = "Counter-clockwise";
      }
    }
  }

  // Update last encoder state
  lastEncoderStateA = currentEncoderStateA;
}











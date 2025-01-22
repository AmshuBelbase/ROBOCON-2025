// Arduino Nano UART Communication for Robot Control

// Motor control pin definitions
int dir1 = 5;     // Direction pin for motor 1
int speed1 = 6;   // Speed control (PWM) pin for motor 1
int dir2 = 4;     // Direction pin for motor 2
int speed2 = 3;   // Speed control (PWM) pin for motor 2

// Variables for motor speeds
int motorSpeedRight = 0;
int motorSpeedLeft = 0;

// Thresholds for object positioning
int thresholdLeft = 256;   // Threshold for left direction
int thresholdRight = 384;  // Threshold for right direction

// Variables for incoming object tracking data
int x_coordinate = 0;  // X-coordinate of the object
int distance = 0;      // Distance of the object from the robot

// Setup function - initializes the Serial communication and sets up pin modes
void setup() {
  Serial.begin(115200); // Start Serial monitor with baud rate 115200
  pinMode(dir1, OUTPUT); // Set direction and speed pins as outputs
  pinMode(dir2, OUTPUT);
  pinMode(speed1, OUTPUT);
  pinMode(speed2, OUTPUT);
}

// Main loop - reads data, processes control logic, and drives the motors accordingly
void loop() {
  // Check if data is available on the Serial input
  if (Serial.available() > 0) {
    // Read incoming data from Serial until newline character ('\n')
    String incomingData = Serial.readStringUntil('\n');
    
    // Find the comma separator in the data string
    int separatorIndex = incomingData.indexOf(',');
    
    // Check if separator is found in incoming data
    if (separatorIndex != -1) {
      // Extract and convert x-coordinate and distance values
      String value1String = incomingData.substring(0, separatorIndex);
      String value2String = incomingData.substring(separatorIndex + 1);

      x_coordinate = value1String.toInt();  // Convert x-coordinate to integer
      distance = value2String.toInt();      // Convert distance to integer
      
      Serial.println(x_coordinate);         // Print the x-coordinate for debugging
      Serial.println(distance);             // Print the distance for debugging
    }
  }

  int forwardSpeed = 180;  // Set forward movement speed for robot
  
  // Control logic based on the object's x-coordinate position
  if (x_coordinate < thresholdLeft) {
    // Object is to the left of the robot
    int turnSpeed = map(x_coordinate, 0, thresholdLeft, 180, 0);  // Calculate turn speed
    motorSpeedLeft = forwardSpeed - turnSpeed;  // Adjust left motor speed
    motorSpeedRight = forwardSpeed;             // Keep right motor at forward speed
    moveLeft(motorSpeedLeft, motorSpeedRight);  // Execute left movement
  } 
  else if (x_coordinate > thresholdRight) {
    // Object is to the right of the robot
    int turnSpeed = map(x_coordinate, thresholdRight, 640, 180, 0);  // Calculate turn speed
    motorSpeedRight = forwardSpeed - turnSpeed;  // Adjust right motor speed
    motorSpeedLeft = forwardSpeed;               // Keep left motor at forward speed
    moveRight(motorSpeedLeft, motorSpeedRight);  // Execute right movement
  } 
  else if (x_coordinate == 0 && distance == 0) {
    // If no valid coordinates, rotate in place to search for the object
    rotateAxis();
  }
  else {
    // Move forward if the object is directly ahead
    moveForward(forwardSpeed, forwardSpeed);
  }
}

// Function to move the robot left by adjusting motor speeds
void moveLeft(int leftSpeed, int rightSpeed) {
  digitalWrite(dir1, LOW);  // Set motor direction
  digitalWrite(dir2, HIGH); // for forward movement
  analogWrite(speed1, constrain(leftSpeed, 0, 255));  // Set speed with constraints
  analogWrite(speed2, constrain(rightSpeed, 0, 255));
}

// Function to move the robot right by adjusting motor speeds
void moveRight(int leftSpeed, int rightSpeed) {
  digitalWrite(dir1, LOW);  // Set motor direction
  digitalWrite(dir2, HIGH); // for forward movement
  analogWrite(speed1, constrain(leftSpeed, 0, 255));  // Set speed with constraints
  analogWrite(speed2, constrain(rightSpeed, 0, 255));
}

// Function to move the robot straight forward
void moveForward(int leftSpeed, int rightSpeed) {
  digitalWrite(dir1, LOW);  // Set motor direction
  digitalWrite(dir2, HIGH); // for forward movement
  analogWrite(speed1, constrain(leftSpeed, 0, 255));  // Set speed with constraints
  analogWrite(speed2, constrain(rightSpeed, 0, 255));
}

// Function to rotate the robot on its axis if no object is detected
void rotateAxis() {
  digitalWrite(dir1, LOW);  // Set both motors to rotate in opposite directions
  digitalWrite(dir2, LOW);
  analogWrite(speed1, 130); // Set rotation speed
  analogWrite(speed2, 130);
}
void setup() {
  Serial.begin(9600);  // Start the serial communication at 9600 baud rate
}

void loop() {
  // Check if data is available to read from the serial port
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n'); // Read the incoming data until a newline

    //Serial.print("Received: ");
    Serial.println(data); // Echo the received data back to the Python script

    
  }
}


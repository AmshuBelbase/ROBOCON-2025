uint8_t rcv = 0;
void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);

  Serial.println("PS4 Receiver Started");
}

void loop() {
  
  if (Serial2.available() >= sizeof(rcv)) {  // Ensure we have a full packet


    Serial2.readBytes((char*)&rcv, sizeof(rcv));
    Serial.println(rcv);

  }
}

#include <QNEthernet.h>

using namespace qindesign::network;

EthernetUDP udp;
IPAddress myIP(192, 168, 1, 101);
uint16_t port = 12345;

// Motor pins
int direc_pin = 9;
int pwm_pin = 8;

// UDP Data
int offset = 0;

// PID variables
float kp_tt = 0.75;    // Proportional gain
float ki_tt = 0;    // Integral gain
float kd_tt = 0.15;   // Derivative 

float previous_error_tt = 0;
float integral_tt = 0;

unsigned long previous_time_tt = 0;
float dt = 0.1;    

int max_pwm = 230;
int count_not_detected = 0;

void setup() {
  pinMode(direc_pin, OUTPUT);
  pinMode(pwm_pin, OUTPUT);
  
  Serial.begin(115200);
  Ethernet.begin();
  Ethernet.setLocalIP(myIP);
  udp.begin(port);
  
  Serial.print("UDP Receiver ready at IP: ");
  Serial.println(Ethernet.localIP());

  previous_time = millis();
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
void loop() {
  // Receiving data
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char packet[64];
    int len = udp.read(packet, sizeof(packet) - 1);
    if (len > 0) {
      packet[len] = '\0';
      int parsed = sscanf(packet, "%d", &offset);
      if (parsed == 1) {
        Serial.printf("Received Offset: %d\n", offset);
      } else {
        Serial.println("Packet Parsing Error");
      }
    }
    udp.flush();
  }
  checkSerialInput();
  
  unsigned long current_time = millis();
  
  if ((current_time - previous_time) >= (dt * 1000)) 
  {
    previous_time = current_time;

    // PID control
    float error = offset; //  offset is error
    integral =integral+ (error * dt);
    float derivative = (error - previous_error) / dt;
    float output = (kp * error) + (ki * integral) + (kd * derivative);
    previous_error = error;

    // Limit output
    output = constrain(output, -max_pwm, max_pwm);

    // Safety: If offset is zero for 5 times, stop motor
    if (offset == 0) {
      count_not_detected++;
      if (count_not_detected >= 5) {
        count_not_detected = 0;
        digitalWrite(direc_pin, LOW);
        analogWrite(pwm_pin, 0);
      }
    }
    else if (offset == 1) { // 1 means centered
      count_not_detected = 0;
      digitalWrite(direc_pin, LOW);
      analogWrite(pwm_pin, 0);
    }
    else {
      count_not_detected = 0;
      digitalWrite(direc_pin, (output > 0) ? LOW : HIGH);
      analogWrite(pwm_pin, abs((int)output));
    }
  }
}

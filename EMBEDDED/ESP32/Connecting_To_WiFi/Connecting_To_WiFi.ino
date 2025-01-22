#include <WiFi.h>  // Include the Wi-Fi library

const char* ssid = "YOUR_SSID";  // Replace with your Wi-Fi network name
const char* password = "YOUR_PASSWORD";  // Replace with your Wi-Fi password

void setup() {
  Serial.begin(115200);  // Start the serial communication at 115200 baud rate
  delay(1000);

  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);  // Start the connection process

  // Wait until the ESP32 is connected to the Wi-Fi network
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());  // Print the local IP address assigned to the ESP32
}

void loop() {
  // Nothing is done here, as we're just interested in connecting to Wi-Fi
}
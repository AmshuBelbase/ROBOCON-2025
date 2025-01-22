#include <WiFi.h>
#include <WebServer.h>

const char *ssid = "ESP32_AP";
const char *password = "abc12345";

void setup() {
  Serial.begin(115200);
  
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid,password);

  delay(2000);

  Serial.println("Access point started");

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP Address: ");
  Serial.println(IP);
  // put your setup code here, to run once:

}

void loop() {

}
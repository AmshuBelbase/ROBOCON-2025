#include <WiFi.h> //Library for WiFi connection
#include <WebServer.h> // Library to handle hTTP requests and setting up webserver on the ESP32


const char* ssid = "ESP32"; // Define the ssid of the network that ESP32 has to conenct to
const char* password = "itsninehertz"; // Define the password of the network the ESP32 has to connect to 


WebServer server(80); // Create a webserver object on port 80 to listen for requests


int x_offset = 0; // Define the x-offset variable to recieve the x_offset through the query parameters
int distance = 0; //Define the distance variable to recieve the distance through the query parameters

void handleOffset() { //Create the handleOffset to be called when the request is made to the /offset path
  if (server.hasArg("x_offset") && server.hasArg("distance")) { //Check if the HTTP rquest has both of these parameters i.e. x_offset and distance
    x_offset = server.arg("x_offset").toInt(); //Extract the data from the HTTP request and convert them from string to int and store them in the respective variable
    distance = server.arg("distance").toInt(); //Extract the data from the HTTP request and convert them from string to int and store them in the respective variable

  
    Serial.print("Received x_offset: "); //Print "Received x_offset: ""
    Serial.println(x_offset); //print the x_offset value recieved and move to the next line
    Serial.print("Received distance: "); // print "Received distance: "
    Serial.println(distance); //print the x_offset value recieved and move to the next line
    server.send(200, "text/plain", "Offsets received"); //Send a response to the client saying Offsets recieved 
  }
  else{
    server.send(200,"text/plain", "Offsets not recieved"); //Else send a response to the client that offsets not recieved 
  }
  
  
}

void setup() {
  Serial.begin(115200); //Start the serial communication at 115200 baud rate
  Serial2.begin(115200, SERIAL_8N1 , 3 , 1); /*Start the serial communication at 115200 baud rate for the second serial port for communication
  with Arduino NANO with pins 3(RX) , 1(TX)*/

  WiFi.begin(ssid, password); // Start the conenection to wifi at the specified ssid and password
  while (WiFi.status() != WL_CONNECTED) { //Run this lopp till the WIFI is connected
    delay(1000); //Have a delay of 1second
    Serial.println("Connecting to Wi-Fi..."); //Print Connecting to WiFi
  }
  Serial.println("Connected to Wi-Fi"); //Print connected to WiFi after we get out of the loop
  Serial.println(WiFi.localIP()); //Print the local IP address of the ESP32 

  server.on("/offset", handleOffset); // Set up a route so that when a GET request is made to the /offset path the handleOffset function is called
  server.begin(); // Start up the WebServer
  Serial.println("Web server started"); //Print "Web server started"
}

void loop() {
  server.handleClient(); //Keeps checking for icoming client requests and processes it
  Serial2.print(x_offset); // the following two lines sends the current values of x_offset and distance over the serial communication (UART) to arduino NANO formatted as "{x_offset},{distance}"
  Serial2.print(","); 
  Serial2.println(distance); 
}

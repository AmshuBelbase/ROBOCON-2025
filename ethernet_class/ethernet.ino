#include <QNEthernet.h>
using namespace qindesign::network;

#pragma pack(push, 1)
struct ControllerData {
    int32_t axis[4];
    int16_t r1;
    int16_t cross;
    int16_t circle;
    int16_t triangle;
    int16_t turn_pwm;
    int16_t bldc_pwm;
};
#pragma pack(pop)

EthernetClient client;
ControllerData data;
const IPAddress serverIP(192, 168, 1, 102);
uint32_t lastSend = 0;
uint32_t timeout = 10000; // 10ms timeout
uint8_t buffer[sizeof(ControllerData)];
size_t bufIndex = 0;
 
void setup() {  
    IPAddress ip(192, 168, 1, 101);
    IPAddress subnet(255, 255, 255, 0);

    Ethernet.begin();
    Ethernet.setLocalIP(ip);
    Ethernet.setSubnetMask(subnet);

    Serial.begin(115200);
    while (!Serial);

    Serial.print("TCP Client started at: ");
    Serial.println(Ethernet.localIP());
    while(!client.connect(serverIP, 10069)) {
        delay(100); 
    }
    client.setNoDelay(true); // Disable Nagle algorithm
}

static uint32_t sent_counter = 0;
static uint32_t received_counter = 0;


void loop() {
    
    // Non-blocking send every 5ms
    if(millis() - lastSend >= 1) {
        client.write(0x01);
        client.flush(); 
        lastSend = millis();
        sent_counter++;
        Serial.printf("Sent Request: %d\n", sent_counter);
        while(!client.available()){}
    }


    // Non-blocking read with timeout
    uint32_t start = millis();
    while(client.available() && (millis() - start < timeout)) {
        buffer[bufIndex++] = client.read();
        
        if(bufIndex == sizeof(ControllerData)) {
            memcpy(&data, buffer, sizeof(ControllerData));
            received_counter++;
            Serial.printf("%d %d %d %d %d | Time: %lums (%d)\n",
                         data.axis[0], data.axis[1], data.axis[2], data.axis[3],
                         data.r1, millis() - lastSend, received_counter);
            bufIndex = 0;
            break;
        }
    }
    
    // Maintain connection
    if(!client.connected()) {
        client.connect(serverIP, 10069);
        client.setNoDelay(true); // Disable Nagle algorithm
    }

    delay(5000);
}


// #include <QNEthernet.h>
// using namespace qindesign::network;

// struct __attribute__((packed)) ControllerData {
//     int32_t axis[4];
//     int16_t r1;
//     int16_t cross;
//     int16_t circle;
//     int16_t triangle;
//     int16_t turn_pwm;
//     int16_t bldc_pwm;
// };

// EthernetClient client;
// ControllerData data;
// const IPAddress serverIP(192, 168, 1, 102);

// void setup() {  
//     IPAddress ip(192, 168, 1, 101);
//     IPAddress subnet(255, 255, 255, 0);

//     Ethernet.begin();
//     Ethernet.setLocalIP(ip);
//     Ethernet.setSubnetMask(subnet);

//     Serial.begin(115200);
//     while (!Serial);

//     Serial.print("TCP Client started at: ");
//     Serial.println(Ethernet.localIP());
// }

// int send_counter = 0;
// int receive_counter = 0;


// void loop() {
//     while (!Serial);
//     static uint32_t lastSend = 0;

//     if (!client.connected()) {
//       Serial.println("Client disconnected. Reconnecting...");
//       if (!client.connect(serverIP, 10069)) {
//         Serial.println("Failed to reconnect.");
//         delay(1000);
//         return;
//       }
//       Serial.println("Reconnected to server."); 
//     } 

    
//     // Send 1-byte request every 5ms
//     if(millis() - lastSend > 5) {
//         client.write(0x01); // Trigger byte
//         lastSend = millis();
//         send_counter++;
//         Serial.printf("Sent Request Count: %d \n", send_counter);
//     }

//     while(!client.available()){}
    
//     // Read response
//     if(client.available() >= sizeof(ControllerData)) {
//         client.read((uint8_t*)&data, sizeof(ControllerData));
//         receive_counter++;
//         Serial.printf("%d %d %d %d %d %d %d %d %d %d Time Taken (%d): %ld\n", data.axis[0], data.axis[1], data.axis[2], data.axis[3], data.r1, data.cross, data.circle, data.triangle, data.turn_pwm, data.bldc_pwm, receive_counter, (millis()-lastSend));
//         // Access fields directly:
//         // data.axis[0], data.r1, etc.
//     }
// }


// #include <ArduinoJson.h>
// #include <QNEthernet.h>

// using namespace qindesign::network;

// EthernetClient client;
// String tempBuffer = "";String msgRecieved = "";

// const IPAddress serverIP(192, 168, 1, 102);

// int axis[4] = {0};
// int r1, cross, circle, triangle;
// int turn_pwm, bldc_pwm;

// void data_receive() {

//   // check if the client is connected or not
//   // if disconnected, try reconnecting until it gets connected

//   if (!client.connected()) {
//     Serial.println("Client disconnected. Reconnecting...");
//     if (!client.connect(serverIP, 10069)) {
//       Serial.println("Failed to reconnect.");
//       delay(1000);
//       return;
//     }
//     Serial.println("Reconnected to server."); 
//   } 
  
//   unsigned long data_receive_time = millis();

//   // send the request to the server to get the data from the server 
//   // Serial.println("Sending GET request...");

//   client.println("GET /data HTTP/1.1");
//   client.println("Host: 192.168.1.102");
//   client.println("Connection: keep-alive"); 
//   client.println(); 
  

//   // wait for response from server until 1 second
//   // if data doesn't come within a second skip the request

//   unsigned long start = millis();
//   while (client.connected() && !client.available()) {
//     if (millis() - start > 1000) {
//       Serial.println("Timeout waiting for response.");
//       return;
//     }
//   }

//   // read response came from the server
//   String response = "";
//   while (client.available()) {
//     response += (char)client.read();
//   }

//   // try to find the body by looking for the end of headers
//   int jsonStart = response.indexOf("\r\n\r\n");

//   // if no headers found, assume the entire response is the body
//   String body;
//   if (jsonStart != -1) {
//     body = response.substring(jsonStart + 4);
//   } else {
//     body = response;  // No headers, treat the entire response as the body
//   }

//   // if the body is empty, print a message and skip this request

//   if (body.length() == 0) {
//     Serial.printf("Received an empty body. \t Time Taken: %ld\n", (millis()-data_receive_time));
//     return;
//   }

//   // Serial.println("+++RESPONSE+++");

//   // parse JSON only if body is found and is non empty

//   ArduinoJson::JsonDocument doc;
//   DeserializationError error = deserializeJson(doc, body);
//   if (error) {
//     Serial.println(response);

//     Serial.print("JSON parse error: ");
//     Serial.println(error.c_str());
//     return;
//   }

//   // extract values from the JSON object

//   for (int i = 0; i < 4; i++) {
//     axis[i] = doc["axis"][i] | 0;
//   }

//   r1 = doc["r1"] | 0;
//   cross = doc["cross"] | 0;
//   circle = doc["circle"] | 0;
//   triangle = doc["triangle"] | 0;
//   turn_pwm = doc["turn_pwm"] | 0;
//   bldc_pwm = doc["bldc_pwm"] | 0;

//   Serial.printf("{\"Axis\":[%d,%d,%d,%d],\"R1\":%d,\"Cross\":%d,\"Circle\":%d,\"Triangle\":%d,\"Turn_pwm\":%d,\"Bldc_pwm\":%d}\t Time Taken: %ld\n",
//     axis[0], axis[1], axis[2], axis[3], r1, cross, circle, triangle, turn_pwm, bldc_pwm, (millis()-data_receive_time));

// }

// void checkEthernetConnection() {
//   if (Ethernet.linkStatus() != LinkON) {
//     Serial.println("Ethernet disconnected. Reinitializing...");

//     IPAddress ip(192, 168, 1, 101);
//     IPAddress subnet(255, 255, 255, 0);

//     Ethernet.begin();
//     Ethernet.setLocalIP(ip);
//     Ethernet.setSubnetMask(subnet);

//     delay(500);

//     if (Ethernet.linkStatus() == LinkON) {
//       Serial.println("Ethernet reconnected.");
//     } else {
//       Serial.println("Still no link.");
//     }
//   }
// }




// void setup() {
//   IPAddress ip(192, 168, 1, 101);
//   IPAddress subnet(255, 255, 255, 0);

//   Ethernet.begin();
//   Ethernet.setLocalIP(ip);
//   Ethernet.setSubnetMask(subnet);

//   Serial.begin(115200);
//   while (!Serial);

//   Serial.print("TCP Client started at: ");
//   Serial.println(Ethernet.localIP());
// }

// unsigned long prevMillis = 0;
// const unsigned long interval = 100;



// void loop() {
//   checkEthernetConnection();

//   for(int i = 0; i<50; i++){
//     Serial.printf("Request: %d", (i+1));
//     data_receive();
//     delay(100);
//   }

//   delay(5000);


//   // unsigned long currMillis = millis();
//   // if (currMillis - prevMillis >= interval) {
//   //   prevMillis = currMillis;
//   //   data_receive();
//   // }

//   // Other tasks
// }

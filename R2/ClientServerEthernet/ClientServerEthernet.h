// A generic TCP client implementation over Ethernet using QNEthernet for Teensy boards.

// This template class facilitates communication with a TCP server to send a request
// and receive a binary-encoded response into a user-defined struct.

// Dependencies:
//  - QNEthernet.h (Teensy 4.x compatible Ethernet library)
//  - IPAddress.h

// Move ClientServerEthernet.h to a subfolder of your Arduino/libraries/ directory:

#ifndef CLIENT_SERVER_ETHERNET_H
#define CLIENT_SERVER_ETHERNET_H
#include <Arduino.h>
#include <QNEthernet.h>
#include <vector>
#include <IPAddress.h>

using namespace qindesign::network;
using namespace std;

// This class handles Ethernet initialization, TCP connection management, and structured data transmission/reception.
// T in template is the data type of the structure being sent/received (typically a struct).
void FUNC() {
  SCB_AIRCR = 0x05FA0004;
}

template <typename T>
class ClientServerEthernet
{
public:
    T *data; // Pointer to the structure where received data is stored.
    EthernetClient client;
    IPAddress ip, subnet, serverIP;

    bool flag = true;
    uint32_t lastSend = 0;
    uint32_t timeout = 10000;  // 10ms timeout
    uint8_t buffer[sizeof(T)]; // Temporary buffer to store incoming bytes.
    size_t bufIndex = 0;

    int sent_counter = 0, received_counter = 0;

    // Initializes with no data pointer and prints a debug message.
    ClientServerEthernet() : data(nullptr)
    {
        Serial.println("Default Ethernet Initialized.");
    }

    // Initializes Ethernet settings, assigns data pointer to store received information, and connects to server.
    ClientServerEthernet(vector<int> ipAddress, vector<int> subnetMask, vector<int> serverAddress, T *inputData) : data(inputData)
    {
        ip = IPAddress(ipAddress[0], ipAddress[1], ipAddress[2], ipAddress[3]);
        subnet = IPAddress(subnetMask[0], subnetMask[1], subnetMask[2], subnetMask[3]);
        serverIP = IPAddress(serverAddress[0], serverAddress[1], serverAddress[2], serverAddress[3]);

        Ethernet.begin();
        Ethernet.setLocalIP(ip);
        Ethernet.setSubnetMask(subnet);

        Serial.print("TCP Client started at: ");
        Serial.println(Ethernet.localIP());

        // Attempt to connect to server
        while (!client.connect(serverIP, 45000))
        {
            delay(100);
        }
        client.setNoDelay(true); // Disable Nagle algorithm

        // Disabling Nagle sends smaller packets immediately, reducing latency but increasing network overhead, as more packets are transmitted more frequently.
    }

public:
    // Sends a request byte (0x01) to the server. Optionally prints debug information.
    void sendRequest(bool debugMode = false)
    {
        client.write(0x01);
        client.flush();
        lastSend = millis();
        sent_counter++;
        if (debugMode)
        {
            Serial.printf("Sent Request: %d\n", sent_counter);
        }

        // Wait for data availability
        while (!client.available())
        {
            if(!client.connected()){
               MaintainConnection();
                client.write(0x01);
                client.flush();
            }
            // Serial.println("N");
        }
    }

    // Ensures TCP connection is maintained. Reconnects if connection is lost.
    void MaintainConnection(bool debugMode = false)
    {
        int startTime = millis();
        while (!client.connected())
        {
            if(millis()-startTime>200){
                Serial.println("New Life");
                FUNC();
            }
            Serial.print(".");
            client.connect(serverIP,45000);
            client.setNoDelay(true); // Disable Nagle algorithm
        }
        Serial.println();
    }

    // Sends request and receives structured data from server.
    // If enough bytes are received to match the size of type T, they are copied into the data pointer. Otherwise, waits until timeout.
    void getData(bool debugMode = false)
    {

        MaintainConnection(debugMode);
        sendRequest(debugMode);

        uint32_t start = millis();
        while (client.available() && (millis() - start < timeout))
        {
            buffer[bufIndex++] = client.read();

            if (bufIndex == sizeof(T))
            {
                memcpy(data, buffer, sizeof(T));
                received_counter++;
                if (debugMode)
                {
                    Serial.printf("%d %d %d %d %d | Time: %lums (%d)\n", data->axis[0], data->axis[1], data->axis[2], data->axis[3], data->r1, millis() - lastSend, received_counter);
                }
                bufIndex = 0;
                break;
            }
        }

        MaintainConnection(debugMode);
    }
};

#endif

// Move ClientServerEthernet.h to a subfolder of your Arduino/libraries/ directory.
#include <ClientServerEthernet.h>

// <<<< IMPORTANT ----

// Ensure the struct is packed with no padding between members.
// This is important for consistent memory layout, especially when sending data over serial or network.
// #pragma pack(1) → No padding (tightest packing).

// int a = 1000; -> size may vary (typically 4 bytes) across systems and compilers
// int16_t b = 1000; -> guaranteed to be 2 bytes across all platforms

// ---- IMPORTANT >>>>

#pragma pack(push, 1) // save current alignment and set to 1 byte
struct ControllerData { 
    int32_t axis[4]; 
    int16_t r1;
    int16_t cross;
    int16_t circle;
    int16_t triangle; 
    int16_t turn_pwm;
    int16_t bldc_pwm;
};
#pragma pack(pop) // restore previous alignment


ControllerData jetdata; // Struct instance to hold incoming controller data

ClientServerEthernet<ControllerData> con; // Instance of the ClientServerEthernet class templated with ControllerData

void setup() {
  Serial.begin(115200);
  
  vector<int> client_ip = {192, 168, 1, 101}; // IP address of this device (client)
  vector<int> server_ip = {192, 168, 1, 102}; // IP address of the server to communicate with
  vector<int> subnet_mask = {255, 255, 255, 0}; // Subnet mask for the network

  // Initialize the Ethernet client-server connection with IPs, subnet, and a pointer to the data structure
  con = ClientServerEthernet<ControllerData>(client_ip, subnet_mask, server_ip, &jetdata);
}

void loop() {
  con.getData();
  Serial.println(jetdata.r1);
  delay(3);
}

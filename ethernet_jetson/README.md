# Client-Server Ethernet Communication for Teensy (QNEthernet)

This project implements a generic TCP client over Ethernet for **Teensy 4.x** boards using the **QNEthernet** library. It enables structured communication with a TCP server using binary-encoded data packets mapped to a C++ struct.

## Features

- Uses `QNEthernet` for fast and reliable Ethernet communication.
- Sends request packets to a server and receives structured binary data.
- Works with custom structs thanks to C++ templates.
- Automatically reconnects to the server if the connection is lost.
- Minimal latency by disabling the Nagle algorithm (`setNoDelay(true)`).

---

## Getting Started

### Requirements

- **Hardware:**
  - Teensy 4.1 or compatible board with Ethernet support.
- **Libraries:**
  - [`QNEthernet`](https://github.com/ssilverman/QNEthernet)
  - `IPAddress.h`
  - `vector` (Standard Template Library)

---

## File Structure

- `CompleteEthernet.ino`: Arduino sketch for setting up the client and polling controller data.
- `ClientServerEthernet.h`: Templated class that handles TCP communication over Ethernet.

---

## Use Libraries Folder

Move ClientServerEthernet.h to a subfolder of your Arduino/libraries/ directory.

```
Arduino/
└── libraries/
    └── ClientServerEthernet/
        └── ClientServerEthernet.h
```

## Struct Format

```cpp
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
```

## How It Works

### Initialization:

```cpp
vector<int> client_ip = {192, 168, 1, 101};
vector<int> server_ip = {192, 168, 1, 102};
vector<int> subnet_mask = {255, 255, 255, 0};
con = ClientServerEthernet<ControllerData>(client_ip, subnet_mask, server_ip, &jetdata);
```

### Polling Loop:

```cpp
void loop() {
    con.getData();
    Serial.println(jetdata.r1);
    delay(3);
}
```

### Communication Protocol:

- Client sends 0x01 as request byte.
- Server responds with binary-packed ControllerData.
- Data is read into a buffer and copied into a struct if fully received.

### Debug Mode

- Enable debug printing in the getData() or sendRequest() methods:

```cpp
con.getData(true); // Prints struct values and timing info
```

### Notes

- This client expects the server to return exactly sizeof(T) bytes per request.

- If fewer bytes are received, the data is ignored until complete.

- Uses memcpy() for binary-safe struct mapping.

### Useful Tips

- Always use fixed-width integer types like int16_t, int32_t to ensure consistent struct sizes across platforms.

- When modifying the ControllerData struct, ensure both client and server are updated identically.

## Developed by - AMSHU BELBASE

### License

© 2025 Amshu Belbase. All rights reserved.

This source code and associated files are the intellectual property of Amshu Belbase.
Unauthorized copying, modification, distribution, or use in any form is strictly prohibited without prior written permission.

For licensing inquiries or permission requests, please contact: amsubelbs@gmail.com

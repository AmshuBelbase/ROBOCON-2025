#include "USBHost_t36.h"

USBHost myusb;
JoystickController joystick1(myusb);
// BluetoothController bluet(myusb, true, "0000"); 
BluetoothController bluet(myusb);

void setup() {
  Serial.begin(9600);
  Serial8.begin(9600);

  Serial.println("PS4 Sender Started");
  myusb.begin();
}

void loop() {
  myusb.Task();  // Process USB communication

  if (joystick1.available()) {
    uint32_t buttons = joystick1.getButtons();  // PS4 Button states (4 bytes)
    Serial8.write((uint8_t*)&buttons, sizeof(buttons));
    // int16_t lx = joystick1.getAxis(0);   // Left stick X (-32768 to 32767)
    // int16_t ly = joystick1.getAxis(1);   // Left stick Y (-32768 to 32767)
    // int16_t rx = joystick1.getAxis(2);   // Right stick X (-32768 to 32767)
    // int16_t ry = joystick1.getAxis(5);   // Right stick Y (-32768 to 32767)



    delay(100);  // Prevent flooding
  }
}

#include "USBHost_t36.h"

USBHost myusb;
USBHub hub1(myusb);
USBHub hub2(myusb);
KeyboardController keyboard1(myusb); 
USBHIDParser hid1(myusb);
USBHIDParser hid2(myusb); 
MouseController mouse1(myusb); 
int user_axis[64];
uint32_t buttons_prev = 0; 

USBDriver *drivers[] = {&hub1, &hub2, &hid1, &hid2};
#define CNT_DEVICES (sizeof(drivers)/sizeof(drivers[0]))
const char * driver_names[CNT_DEVICES] = {"Hub1","Hub2", "HID1", "HID2"};
bool driver_active[CNT_DEVICES] = {false, false, false, false};

USBHIDInput *hiddrivers[] = {&mouse1, &keyboard1};
#define CNT_HIDDEVICES (sizeof(hiddrivers)/sizeof(hiddrivers[0]))
const char * hid_driver_names[CNT_DEVICES] = {"Mouse1", "KB1"};
bool hid_driver_active[CNT_DEVICES] = {false, false};
bool show_changed_only = false;

void setup()
{
  while (!Serial) ; // wait for Arduino Serial Monitor
  Serial.println("\n\nUSB Host Testing");
  Serial.println(sizeof(USBHub), DEC);
  myusb.begin(); 
}

void loop()
{
  myusb.Task();

  for (uint8_t i = 0; i < CNT_DEVICES; i++) {
    if (*drivers[i] != driver_active[i]) {
      if (driver_active[i]) {
        Serial.printf("*** Device %s - disconnected ***\n", driver_names[i]);
        driver_active[i] = false;
      } else {
        Serial.printf("*** Device %s %x:%x - connected ***\n", driver_names[i], drivers[i]->idVendor(), drivers[i]->idProduct());
        driver_active[i] = true;

        const uint8_t *psz = drivers[i]->manufacturer();
        if (psz && *psz) Serial.printf("  manufacturer: %s\n", psz);
        psz = drivers[i]->product();
        if (psz && *psz) Serial.printf("  product: %s\n", psz);
        psz = drivers[i]->serialNumber();
        if (psz && *psz) Serial.printf("  Serial: %s\n", psz);
      }
    }
  }

  for (uint8_t i = 0; i < CNT_HIDDEVICES; i++) {
    if (*hiddrivers[i] != hid_driver_active[i]) {
      if (hid_driver_active[i]) {
        Serial.printf("*** HID Device %s - disconnected ***\n", hid_driver_names[i]);
        hid_driver_active[i] = false;
      } else {
        Serial.printf("*** HID Device %s %x:%x - connected ***\n", hid_driver_names[i], hiddrivers[i]->idVendor(), hiddrivers[i]->idProduct());
        hid_driver_active[i] = true;

        const uint8_t *psz = hiddrivers[i]->manufacturer();
        if (psz && *psz) Serial.printf("  manufacturer: %s\n", psz);
        psz = hiddrivers[i]->product();
        if (psz && *psz) Serial.printf("  product: %s\n", psz);
        psz = hiddrivers[i]->serialNumber();
        if (psz && *psz) Serial.printf("  Serial: %s\n", psz); 
        #if 0  // In list in older, newer code should support the N Key rollover directly
        if (hiddrivers[i] == &keyboard1) {
          if (keyboard1.idVendor() == 0x04D9) {
            Serial.println("Gigabyte vendor: force boot protocol");
            // Gigabyte keyboard
            keyboard1.forceBootProtocol();
          }
        }
        #endif
      }
    }
  }

  if (mouse1.available()) { 
    Serial.print(",  mouseX = ");
    Serial.print(mouse1.getMouseX());
    Serial.print(",  mouseY = ");
    Serial.print(mouse1.getMouseY()); 
    Serial.println();
    mouse1.mouseDataClear();
  }
   
}
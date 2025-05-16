#include "USBHost_t36.h"
#include <VescUart.h>
VescUart UART;

// PS4 connection
USBHost myusb;
JoystickController joystick1(myusb);
BluetoothController bluet(myusb, true, "0000");   // Version does pairing to device
// BluetoothController bluet(myusb); 

uint32_t buttons;  

int turntable_pwm = 18;
int turntable_dir = 17;
int rpm = 0;

int L2_value = 0;
int R2_value = 0;




void setup() {
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  pinMode(turntable_pwm, OUTPUT);
  pinMode(turntable_dir, OUTPUT);

  digitalWrite(turntable_dir, LOW);
  analogWrite(turntable_pwm, 0);

  Serial.begin(200000);
  Serial7.begin(115200);

  Serial.println("\n\nUSB Host Testing - Joystick Bluetooth");

  if (CrashReport) Serial.print(CrashReport);
  myusb.begin();

  UART.setSerialPort(&Serial7);

  analogWriteResolution(14);
  analogWriteFrequency(0, 9000);
}

void loop() {
  myusb.Task();
  if (joystick1.available()) {
    // isJoystick = true;

    buttons = joystick1.getButtons();
    // Serial.printf("Button: %d", buttons);


    L2_value = joystick1.getAxis(3);  // Axis 2 is L2
    R2_value = joystick1.getAxis(4);  // Axis 5 is R2

    // Serial.printf("L2: %d\n", L2_value);
    // Serial.printf("R2: %d\n", R2_value);
  }
  Serial.printf("L2: %d, R2: %d, RPM: %d, Button: %d\n", L2_value*16, R2_value, rpm, buttons);


  if (buttons == 131072){       // right
    digitalWrite(turntable_dir, HIGH);
    analogWrite(turntable_pwm, 255*16);
  }
  else if(buttons == 524288){    // left
    digitalWrite(turntable_dir, LOW);
    analogWrite(turntable_pwm, 255*16);
  }
  // else if (buttons != 131136 || buttons != 524352){
  //   digitalWrite(turntable_dir, LOW);
  //   analogWrite(turntable_pwm, 0);
  // }
 
  else if (buttons == 65536 && flag_timer[2]==1){       // up
    buttons = 0;
    if (rpm ==0){
      rpm = 500;
    }
    // Serial.println("rpm up");
    rpm += 500;
    UART.setRPM(rpm);
    flag_timer[2] = 0;
  }
  else if (buttons == 65552 && flag_timer[2]==1){       // up
    buttons = 0;
    if (rpm ==0){
      rpm = 500;
    }
    // Serial.println("rpm up");
    rpm += 100;
    UART.setRPM(rpm);
    flag_timer[2] = 0;
  }
  else if (buttons == 262144 && flag_timer[2]==1){      // down
    buttons = 0;
    rpm -= 500;
    UART.setRPM(rpm);
    flag_timer[2] = 0;
  }
  else if (buttons == 262160 && flag_timer[2]==1){      // down
    buttons = 0;
    rpm -= 100;
    UART.setRPM(rpm);
    flag_timer[2] = 0;
  }

  // static unsigned long lastUpdate = 0;
  // unsigned long currentTime = millis();

  // if (buttons == 160 && R2_value != 0) {
  //     if (currentTime - lastUpdate > 200) {  
  //         rpm += R2_value / 10;  
  //         lastUpdate = currentTime;
  //     }
  //     UART.setRPM(rpm);
  // }

  // else if (buttons == 160 && R2_value != 0){
  //   rpm += R2_value/5;
  //   UART.setRPM(rpm);
  // }
  if(millis()-lastTime>=1000)
  {
    for(int i=0;i<20;i++)
    {flag_timer[i]=1;
    }
    lastTime=millis();
    // Serial.println(flag_timer[2]);
  }
}

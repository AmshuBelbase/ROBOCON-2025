#include <Encoder.h>
#include "Wire.h"
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <IntervalTimer.h>
#include "USBHost_t36.h"

int auto_flag = 1; // 1 for autonomous, 0 for manual

Encoder myEnc[3] = {Encoder(36, 33), Encoder(38, 37), Encoder(40, 39)};

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire2); // 0x28 is the default I2C address

Encoder encoder1(19,18); // Pins for encoder 1
Encoder encoder2(22,23); // Pins for encoder 2

volatile int currentTick1 = 0; // Position for encoder 1
volatile int currentTick2 = 0; // Position for encoder 2

volatile int oldTick1 = 0;
volatile int oldTick2 = 0;

volatile int deltaTick1 = 0;
volatile int deltaTick2 = 0;

volatile float globalX = 0;
volatile float globalY = 0;

volatile float oldx = 0;
volatile float oldy = 0;
volatile float oldangle = 0;

// volatile float speedX = 0;
// volatile float speedY = 0;

volatile float deltaX = 0;
volatile float deltaY = 0;

volatile float globalTheta = 0;
volatile float oldTheta = 0;
volatile float deltaTheta = 0;
volatile float virtual_global_Theta = 0;

volatile float yawVel = 0;

volatile float accX=0;
volatile float accY=0;

volatile float lastTheta = 0.1;
volatile float lastX = 0.1;
volatile float lastY = 0.1;
unsigned long mil = millis();


const float wheelDiameter = 5.8; 
const int encoderResolutionX = 2280;
const int encoderResolutionY = 2280;
const float CY= (PI * wheelDiameter / encoderResolutionY );
const float CX= (PI * wheelDiameter / encoderResolutionX );

const float L = 17.6; //12.20
const int B = 12.2;  //19.30

// PS4 connection
USBHost myusb;            // initializes and manages the USB host port, enabling Teensy to detect and commuincate with USB bluetooth dongle
USBHIDParser hid1(myusb); // works behind the scenes to parse HID data that comes from the PS4 controller, such as joystick movements and button presses.
JoystickController joystick(myusb);
// BluetoothController bluet(myusb, true, "0000");
BluetoothController bluet(myusb);   // Version does pairing to device

// coordinates of joystick (x,y -> right joystick; leftX -> left joystick)
int x = 0, y = 0, leftX = 0;
int PWM[3] = { 4, 5, 3 };
int DIR[3] = { 6, 7, 2 };

long currentCounts[3] = {0, 0, 0};
volatile long lastCount[3] = {0, 0, 0};
volatile double rpm[3] = {0, 0, 0}; // Stores the calculated RPM
long positionChange[3] = {0, 0, 0};

// pid constants
float kp[3] = {9.0, 9.0, 9.0};
float ki[3] = {165.0, 165.0, 165.0};
float kd[3] = {0.5, 0.5, 0.5};

volatile float sp[3] = {0, 0, 0};
float pid[3] = {0.0, 0.0, 0.0};
float err[3] = {0.0, 0.0, 0.0};
float prev_err[3] = {0.0, 0.0, 0.0};
float integ[3] = {0.0, 0.0, 0.0};
float der[3] = {0.0, 0.0, 0.0};

float max_rpm = 250;

IntervalTimer timer; // Timer object for periodic execution

int temp_leftX ,temp_x ,temp_y;

// sensors_event_t event, linearAccelData;

void calculatePID()
{ 
  odometry();

  // input();
  myusb.Task(); // Handle USB host tasks

  if (joystick.available())
  {
    // Left Stick values (axes 0 and 1)
    int leftStickX = joystick.getAxis(0);
    temp_leftX = map(leftStickX, 0, 255, -100, 100);

    // Right Stick values (axes 2 and 5)
    int rightStickX = joystick.getAxis(2);
    temp_x = map(rightStickX, 0, 255, -100, 100);

    int rightStickY = joystick.getAxis(5);
    temp_y = map(rightStickY, 0, 255, 100, -100);
    // temp_y = -temp_y;

    // Serial.printf("x:%d\n", temp_x);
    // Serial.printf("y:%d", temp_y);
    // Serial.printf("leftX:%d",temp_leftX);

    // to ignore small joystick values
    if (abs(temp_x) < 5)
      temp_x = 0;
    if (abs(temp_y) < 5)
      temp_y = 0;
    if (abs(temp_leftX) < 5)
      temp_leftX = 0;

    if (auto_flag == 0)
    { // manual mode
      x = temp_x;
      y = temp_y;
      leftX = temp_leftX;
    }
    else
    { // autonomous mode
      if (temp_x != 0 || temp_y != 0 || temp_leftX != 0)
      { // if any joystick is moved then switch to manual mode
        auto_flag = 0;
        x = 0;
        y = 0;
        leftX = 0;
      }
    }
  }

  if(globalX!=lastX || globalY!=lastY || globalTheta!=lastTheta || (millis()-mil)>800)
  {
    Serial.print(auto_flag);
    Serial.print(",");
    Serial.print(globalX);
    Serial.print(",");
    Serial.print(globalY);
    Serial.print(",");
    Serial.print(globalTheta);
    Serial.print(",");
    Serial.print(yawVel); 
    Serial.print(",");
    Serial.print(accX); 
    Serial.print(",");
    Serial.println(accY); 
    // Serial.print(",");
    // Serial.print(auto_flag);
    // Serial.print(",");
    // Serial.print(lastX);
    // Serial.print(",");
    // Serial.print(lastY);
    // Serial.print(",");
    // Serial.print(lastTheta);
    // Serial.print(",");
    // Serial.println((millis()-mil));

    lastX=globalX;
    lastY=globalY;
    lastTheta=globalTheta;
    mil = millis();
  }

  if (Serial.available())
  {
    String data = Serial.readStringUntil('\n'); // Read the incoming data

    int sx, sy, sangle;

    int len = data.length();
    String v1 = "";
    String v2 = "";
    String v3 = "";
    int c = 0;
    for (int i = 0; i < len; i++)
    {
      if (data[i] == ',')
      {
        c++;
      }
      else
      {
        if (c == 0)
        {
          v1 += data[i];
        }
        else if (c == 1)
        {
          v2 += data[i];
        }
        else
        {
          v3 += data[i];
        }
      }
    }

    sx = v1.toInt();
    sy = v2.toInt();
    sangle = v3.toInt();

    if (auto_flag == 1)
    {
      x = sx;
      y = sy;
      // Print received values for debugging
      // Serial.print("X: "); Serial.print(x);  // Print with 2 decimal places
      // Serial.print(" Y: "); Serial.print(y);
      // delay(5000);
      // Serial.print(" Angle: "); Serial.println(angle, 2);
    }
  }

  sp[0] = ((x) * (-0.67) + (y) * 0 + (leftX) * (-0.33));
  sp[1] = ((x) * (0.33) + (y) * (-0.57) + (leftX) * (-0.33));
  sp[2] = ((x) * (0.33) + (y) * (0.57) + (leftX) * (-0.33));

  sp[0] = map(sp[0], -72, 72, -max_rpm, max_rpm);
  sp[1] = map(sp[1], -72, 72, -max_rpm, max_rpm);
  sp[2] = map(sp[2], -72, 72, -max_rpm, max_rpm);

  // Calculate RPM
  for (int i = 0; i < 3; i++)
  {
    currentCounts[i] = myEnc[i].read();
    positionChange[i] = currentCounts[i] - lastCount[i];
    rpm[i] = (positionChange[i] / 1300.0) * (60 * (1000.0 / 75));
    lastCount[i] = currentCounts[i];
  }

  // Serial.printf(" rpm1:%f", rpm[0]);
  // Serial.printf(" rpm2:%f", rpm[1]);
  // Serial.printf(" rpm3:%f\n", rpm[2]);

  // PID Control
  for (int i = 0; i < 3; i++)
  {
    err[i] = sp[i] - rpm[i];
    integ[i] = integ[i] + (err[i] * 0.075);
    der[i] = (err[i] - prev_err[i]) / 0.075;

    pid[i] = (kp[i] * err[i]) + (ki[i] * integ[i]) + (kd[i] * der[i]);
    prev_err[i] = err[i];

    pid[i] = constrain(pid[i], -16383, 16383);
  }

  // Set motor speeds based on calculated velocities
  // runMotor(PWM[0], DIR[0], pid[0]);
  // runMotor(PWM[1], DIR[1], pid[1]);
  // runMotor(PWM[2], DIR[2], pid[2]);
}

void runMotor(int EN, int IN, float speed)
{
  int pwmValue = abs(speed);
  // int pwmValue = constrain(abs(speed),0,200);
  // int pwmValue = map(abs(speed), 0, 127, 0, 16383);

  if (speed > 0)
  { // to check direction: if +ve - HIGH, else LOW
    digitalWrite(IN, HIGH);
  }
  else if (speed < 0)
  {
    digitalWrite(IN, LOW);
  }
  else
  {
    pwmValue = 0;
  }
  analogWrite(EN, pwmValue);
}

void setup()
{
  Wire2.begin();
  Serial.begin(115200);


  // Motor control pins setup
  for (int i = 0; i < 3; i++)
  {
    pinMode(PWM[i], OUTPUT);
    pinMode(DIR[i], OUTPUT);
  }

  // Initialize motor to stop
  for (int i = 0; i < 3; i++)
  {
    analogWrite(PWM[i], 0);
    digitalWrite(DIR[i], LOW);
  }

  analogWriteResolution(14);
  analogWriteFrequency(0, 9000);

  // UART.setSerialPort(&Serial1);
  Serial.println("\n\nUSB Host Testing - Joystick Bluetooth");
  if (CrashReport)
    Serial.print(CrashReport);
  myusb.begin();
  myusb.Task();

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  // while (!Serial)
  //   delay(10); // Wait for Serial Monitor

  // Initialize BNO055
  while (!bno.begin())
  {
    Serial.println("BNO055 not detected. Check connections!");
  }
  Serial.println("BNO055 detected!");

  // sensors_event_t orientationData , angVelocityData , linearAccelData;
  timer.begin(calculatePID, 75000);

  // Optional: Configure to NDOF mode (for fused orientation data)
  bno.setExtCrystalUse(true);
}

void loop()
{
  // odometry();
  // delay(10);
  // Serial.print(globalX);
  // Serial.print(",");
  // Serial.println(globalY);
}

void odometry()
{
  
  sensors_event_t event, linearAccelData;
  // sensors_event_t event;
  // bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  // bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&event);
  // Save old tick values
  oldTick1 = currentTick1;
  oldTick2 = currentTick2;

  // Read encoder positions
  currentTick1 = encoder1.read();
  currentTick2 = encoder2.read();

  // Calculate change in encoder ticks
  deltaTick1 = currentTick1 - oldTick1;
  deltaTick2 = currentTick2 - oldTick2;
  // Serial.print(currentTick1);
  // Serial.print(" ,");
  // Serial.println(currentTick2);
  // // Save the previous virtual global theta
  oldTheta = virtual_global_Theta;

  // Get the new global theta from the sensor
  globalTheta = -event.orientation.x; // BNO055 provides the heading angle
  globalTheta += 360;

  accX = linearAccelData.acceleration.x;
  accY = linearAccelData.acceleration.y;
  // yawVel = angVelocityData.gyro.z;

  // Serial.println(globalTheta);

  // Calculate the change in theta from the IMU
  deltaTheta = globalTheta - oldTheta;

  // Handle wrapping of deltaTheta within -180° to 180°
  while (deltaTheta > 180)
  {
    deltaTheta -= 360;
  }
  while (deltaTheta < -180)
  {
    deltaTheta += 360;
  }

  // Update virtual global theta and normalize it
  virtual_global_Theta += deltaTheta;

  // Convert the virtual global theta to radian
  float ThetaRad = virtual_global_Theta * PI / 180;

  // Compute local displacements
  deltaX = (deltaTick1 * CX) - (L * deltaTheta * PI / 180);
  deltaY = (deltaTick2 * CY) + (B * deltaTheta * PI / 180);

  // Convert local displacements to global coordinates
  double cos_theta = cos(ThetaRad);
  double sin_theta = sin(ThetaRad);

  double delta_x_global = cos_theta * deltaX - sin_theta * deltaY;
  double delta_y_global = sin_theta * deltaX + cos_theta * deltaY;

  // Update global positions
  globalX += delta_x_global;
  globalY += delta_y_global;
}
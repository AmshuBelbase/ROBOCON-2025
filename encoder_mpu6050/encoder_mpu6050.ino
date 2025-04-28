#include <Encoder.h>
#include "Wire.h"
#include <MPU6050_light.h>
#include <math.h>

MPU6050 mpu(Wire2);

Encoder encoder1(0,1);// Pins for encoder 1
Encoder encoder2(3,2);// Pins for encoder 2

volatile int currentTick1 = 0; // Position for encoder 1
volatile int currentTick2 = 0; // Position for encoder 2

volatile int oldTick1 = 0; 
volatile int oldTick2 = 0; 

volatile int deltaTick1 = 0; 
volatile int deltaTick2 = 0; 

volatile float globalX = 0; 
volatile float globalY = 0; 

volatile float deltaX = 0; 
volatile float deltaY = 0;

volatile float globalTheta=0;
volatile float oldTheta=0;
volatile float deltaTheta=0;

const float wheelDiameter = 5.6; 
const int encoderResolution = 2400;
const float C= (PI * wheelDiameter / encoderResolution );
const int L= 51.5;
const int B= 49;

void setup() {
  Serial.begin(9600);
  Wire2.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);

  while (status != 0) {
    Serial.println(F("MPU6050 connection failed!"));
    delay(1000);
  }

  Serial.println(F("Calculating offsets"));
  delay(1000);
  mpu.calcOffsets();
  Serial.println(F("Offsets calculated!"));

}

void loop() {
  odometry();
  delay(10);
  Serial.print("X axis:");
  Serial.println(globalX);
  Serial.print("Y axis:");
  Serial.println(globalY);
}

void odometry(){
  mpu.update();
  oldTick1=currentTick1;
  oldTick2=currentTick2;

  currentTick1 = encoder1.read(); 
  currentTick2 = encoder2.read();
  Serial.println(currentTick1);
  Serial.println(currentTick2);

  deltaTick1=currentTick1-oldTick1;
  deltaTick2=currentTick2-oldTick2;

  oldTheta = globalTheta;
  globalTheta = (mpu.getAngleZ()) * (-1);
  Serial.println(globalTheta, 2);
  volatile int ThetaRad= globalTheta * PI / 180 ;
  deltaTheta = (globalTheta - oldTheta) * PI / 180 ; 

  deltaX=(deltaTick1 * C) - (L * deltaTheta);
  deltaY=(deltaTick2 * C) - (B * deltaTheta);

  double cos_theta = cos(ThetaRad);
  double sin_theta = sin(ThetaRad);

  double delta_x_global = cos_theta * deltaX - sin_theta * deltaY;
  double delta_y_global = sin_theta * deltaX + cos_theta * deltaY;

  globalX += delta_x_global;
  globalY += delta_y_global;
}


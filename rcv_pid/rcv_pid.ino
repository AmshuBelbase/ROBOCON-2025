#include <Encoder.h>
#include <IntervalTimer.h> 

Encoder myEnc[2] = {Encoder(7,6), Encoder(10,9)};   //right - left

// Encoder myEnc(9, 10);

// Motor driver pins
// int PWM = 22;
// int DIR = 20;
int PWM[2] = {19, 18};
int DIR[2] = {17, 16};
// #define motor2PWM 6
// #define motor2DIR 7

//pid constants
float kp = 0.0;   //kp=35
float ki = 0.0;   //ki=0.5
float kd = 0.0;   //kd=0.3

// long currentCounts[2] = {0,0};
long currentCounts[2] = {0,0};

volatile float sp[2] = {0,0};
float pid[2] = {0.0, 0.0};
float err[2] = {0.0, 0.0};
float prev_err[2] = {0.0, 0.0};
float integ[2] = {0.0, 0.0};
float der[2] = {0.0, 0.0}; 

// double prevError1 = 0, prevError2 = 0;          // double error1, error2, prevError1 = 0, prevError2 = 0;
// int sp = 0;
// float integ1=0.0, integ2=0.0;
// float der1=0.0, der2=0.0;
// float pid1 = 0.0, pid2 = 0.0;

IntervalTimer timer;

void setup() {
  Serial.begin(9600);

  pinMode(13, OUTPUT);
  // digitalWrite(13, HIGH);

  // Motor control pins setup
  for (int i = 0; i < 2; i++) {
    pinMode(PWM[i], OUTPUT);
    pinMode(DIR[i], OUTPUT);
}


  // Initialize motor to stop
  for (int i = 0; i < 2; i++) {
    analogWrite(PWM[i], 0);
    digitalWrite(DIR[i], LOW);
}

  analogWriteResolution(14);
  analogWriteFrequency(0, 9000);

  timer.begin(calcPID, 10000);

}

void input() {
  if (Serial.available() > 0) {       // 0500.20.30
  
  String input = Serial.readString();

  kp = input.substring(0,3).toFloat();
  ki = input.substring(3,6).toFloat();
  kd = input.substring(6,9).toFloat();
  sp[0] = input.substring(9).toInt() / 360.0 * 535;
  }
  sp[1] = -1*sp[0];
// Serial.printf(" kp:%f", kp);
// Serial.printf(" ki:%f", ki);
// Serial.printf(" kd:%f", kd);
// Serial.printf(" sp:%f\n", sp);
}

void calcPID() {
  // unsigned long startTime = micros();

  input();

  for (int i = 0; i < 2; i++){
    currentCounts[i] = myEnc[i].read();
  } 
  // Serial.printf("m1:%d", currentCounts[0]);
  // Serial.printf("m2:%d", currentCounts[1]);
  // currentCounts1 = myEnc1.read();
  // long currentCounts2 = myEnc2.read();

  for (int i=0; i<2; i++){
    err[i] = sp[i] - currentCounts[i];
    integ[i] = integ[i] + (err[i]*0.010);   
    der[i] = (err[i]-prev_err[i])/0.010;

    pid[i] = (kp*err[i]) + (ki*integ[i]) + (kd*der[i]);
    prev_err[i] = err[i];

    pid[i] = constrain(pid[i], -16383, 16383);
  }


  // float err[i] = sp - currentCounts[i];
  // integ1 = integ1 + (err1*0.075);
  // der1 = (err1 - prevError1)/0.075;

  // pid1 = (kp*err1) + (ki*integ1) + (kd*der1);
  // prevError1 = err1;


  runMotor(PWM[0], DIR[0], pid[0]);
  runMotor(PWM[1], DIR[1], pid[1]);
  // digitalWrite(motor1DIR, (pid1 <= 0 ? LOW : HIGH));
  // analogWrite(motor1PWM, abs(pid1));

Serial.printf(" sp1:%f", sp[0]);
Serial.printf(" ticks1:%d", myEnc[0].read());
Serial.printf(" sp2:%f", sp[1]);
Serial.printf(" ticks2:%d\n", myEnc[1].read());

}

void runMotor(int motorPWM, int motorDir, float speed) {
  int pwm = abs(speed);
  pwm = constrain(pwm, 0, 16383);
  if (speed > 0) {      //to check direction: if +ve - HIGH, else LOW

    digitalWrite(motorDir, HIGH);
  } else if (speed < 0) {
    digitalWrite(motorDir, LOW);
   // speed = -speed;
  } else {
    pwm = 0;
  }
  analogWrite(motorPWM, pwm);
}

void loop() {

}



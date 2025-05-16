#include <Encoder.h>
#include <IntervalTimer.h> 

Encoder myEnc1(11,12);
// Encoder myEnc2(9, 8);

// Motor driver pins
int motor1PWM = 22;
int motor1DIR = 20;
// #define motor2PWM 6
// #define motor2DIR 7

//pid constants
float kp = 0.0;   //kp=35
float ki = 0.0;   //ki=0.5
float kd = 0.0;   //kd=0.3

double prevError1 = 0, prevError2 = 0;          // double error1, error2, prevError1 = 0, prevError2 = 0;
int sp = 0;
float integ1=0.0, integ2=0.0;
float der1=0.0, der2=0.0;
float pid1 = 0.0, pid2 = 0.0;

IntervalTimer timer;

void setup() {
  Serial.begin(9600);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  // Motor control pins setup
  pinMode(motor1PWM, OUTPUT);
  pinMode(motor1DIR, OUTPUT);
  // pinMode(motor2PWM, OUTPUT);
  // pinMode(motor2DIR, OUTPUT);

  // Initialize motors to stop
  analogWrite(motor1PWM, 0);
  digitalWrite(motor1DIR, LOW);
  // analogWrite(motor2PWM, 0);
  // digitalWrite(motor2DIR, LOW);

  analogWriteResolution(14);
  analogWriteFrequency(0, 9000);

  timer.begin(calcPID, 75000);

}

void input() {
  if (Serial.available() > 0) {       // 0350.50.30
  
  String input = Serial.readString();

  kp = input.substring(0,3).toFloat();
  ki = input.substring(3,6).toFloat();
  kd = input.substring(6,9).toFloat();
  sp = input.substring(9).toInt() / 360.0 * 535;
  }
//   Serial.printf(" kp:%f\n", kp);
// Serial.printf(" ki:%f", ki);
// Serial.printf(" kd:%f", kd);
// Serial.printf(" sp:%f", sp);
}

void calcPID() {
  // unsigned long startTime = micros();

  input();
 
  long currentCounts1 = myEnc1.read();
  // long currentCounts2 = myEnc2.read();

  // PID Control (for M1)
  float err1 = sp - currentCounts1;
  integ1 = integ1 + (err1*0.075);
  der1 = (err1 - prevError1)/0.075;

  pid1 = (kp*err1) + (ki*integ1) + (kd*der1);
  prevError1 = err1;

  // PID Control (for M2)
  // float err2 = sp - currentCounts2;
  // integ2 = integ2 + (err2*0.075);
  // der2 = (err2 - prevError2)/0.075;

  // pid2 = (kp*err2) + (ki*integ2) + (kd*der2);
  // prevError2 = err2;

  runMotor(motor1PWM,motor1DIR,pid1);
  // digitalWrite(motor1DIR, (pid1 <= 0 ? LOW : HIGH));
  // analogWrite(motor1PWM, abs(pid1));
  // runMotor(motor2PWM,motor2DIR,pid2);

  Serial.print("sp:");
  Serial.print(sp);
  Serial.print("ticks:");
  Serial.println(myEnc1.read());

}

void runMotor(int motorPWM, int motorDir, float speed) {
  int pwm = abs(speed);
  pwm = constrain(pwm, 0, 16383);
  if (speed > 0) {      //to check direction: if +ve - HIGH, else LOW

    digitalWrite(motorDir, HIGH);
  } else if (speed < 0) {
    digitalWrite(motorDir, LOW);
    speed = -speed;
  } else {
    pwm = 0;
  }
  analogWrite(motorPWM, pwm);
}

void loop() {

}



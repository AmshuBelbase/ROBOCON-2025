int roll_dir=5;
int roll_pwm=2;

int feed_dir=8;
int feed_pwm=7;

int frame_dir=3;
int frame_pwm=4;
int inp=-1;

void setup() {
 pinMode(roll_dir,OUTPUT);
 pinMode(roll_pwm,OUTPUT);
 pinMode(feed_dir,OUTPUT);
 pinMode(feed_pwm,OUTPUT);
 pinMode(frame_dir,OUTPUT);
 pinMode(frame_pwm,OUTPUT);
 Serial.begin(9600);
}

void loop() {
    // put your main code here, to run repeatedly:
  if (Serial.available()) {
  inp = Serial.parseInt();
 Serial.print("u entered:");
Serial.println(inp);
  if(inp==5){
    digitalWrite(frame_dir,LOW);//frame going down
 // Call the smooth start-stop function

  smoothStartAndLand(frame_pwm, 45, 100, 800); // PWM 0-255, 300ms fast start, 800ms landing
  //roller up 
  digitalWrite(roll_dir,LOW);
  analogWrite(roll_pwm,60);
    delay(200);
    //dribble
  digitalWrite(roll_dir,HIGH);
  analogWrite(roll_pwm,255);
  delay(400);
  digitalWrite(roll_dir,LOW);
  analogWrite(roll_pwm,255);
  delay(1500);
  analogWrite(roll_pwm,0);
 // upar jaega frame
  digitalWrite(frame_dir,HIGH);//frame going up
smoothStartAndLand(frame_pwm, 45, 100, 800);
// feeder start
digitalWrite(feed_dir,LOW);
analogWrite(feed_pwm,255);
delay(2500);
analogWrite(feed_pwm,0);
inp=-1;

  }

}


void smoothStartAndLand(int frame_pwm, int maxPWM, int boostMs, int landingMs) {
  //  Quick boost to max speed

  analogWrite(frame_pwm, maxPWM);
  delay(boostMs);

  //  Gradually slow down using easing
  int steps = 50; // Number of steps to make it smooth
  for (int i = 0; i <= steps; i++) {
    //  convert or map 0to 1;
    float t = (float)i / steps;
    // Ease-out (cubic) – starts fast, slows down
    float eased = 1 - pow(1 - t, 3);
    int pwmVal = int(maxPWM * (1 - eased)); // Reverse ramp down
    analogWrite(frame_pwm, pwmVal);
    delay(landingMs / steps);// ek ek step ka delay 
  }

  analogWrite(frame_pwm, 0); // Stop the motor
}




#include <Encoder.h>
#include <stdio.h>
IntervalTimer pos_pid_timer;
IntervalTimer serial_input_timer;
Encoder enc1(9, 8);

int shooter_rotation_pwm = 1;
int shooter_rotation_dir = 0;
float shooter_rotation_cpr = 17500;
volatile float kP = 80, kI = 1, kD = 10;

volatile int error_offset = 0;

float ap = 0.0;
volatile float sp_angle = 0.0;
//char dir=0;
void setup() {
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Serial.begin(2000000);
  pinMode(shooter_rotation_pwm, OUTPUT);
  pinMode(shooter_rotation_dir, OUTPUT);

  analogWriteResolution(14);
  serial_input_timer.begin(serial_input,10000);
  pos_pid_timer.begin(pos_pid, 10000);

  pos_pid_timer.priority(1);
  serial_input_timer.priority(0);
}
void setPosition(int speed) {
  //Serial.printf("speed_dir: %d",speed<0?LOW:HIGH);

  digitalWrite(shooter_rotation_dir, speed < 0 ? LOW : HIGH);

  //speed=abs(speed)>=1920?1920:abs(speed);
  //Serial.printf("speed_drive: %d",abs(speed));
  analogWrite(shooter_rotation_pwm, abs(speed));
}

float sp = 0;
float err = 0, der = 0, integ = 0;
float speed_m = 0;
volatile float last_err = 0;
volatile int lastTime=0;
char inputBuffer[20];
int inputIndex = 0;
volatile bool track=false;


void serial_input() {
    while (Serial.available() > 0) {
        char received = Serial.read();
        if (received == '\n') { // End of input
            inputBuffer[inputIndex] = '\0'; // Null-terminate
             if(inputBuffer!="\n")
            {error_offset = atoi(inputBuffer); // Convert to integer
            // sp_angle = shooting_angle * 6.25;
            inputIndex = 0;}// Reset buffer
        } else if (inputIndex < sizeof(inputBuffer) - 1) {
            inputBuffer[inputIndex++] = received;
        }
    }
  
    

}

void pos_pid() {
  ap = enc1.read();
  sp = sp_angle * shooter_rotation_cpr / 360.0;

  err = error_offset;
  der = (err - last_err) / 0.01;
  integ += (err - last_err) * 0.01;
  last_err = err;

  speed_m = kP * err + kI * integ + kD * der;

  // Serial.printf("speed: %f   ",speed_m);
  // Serial.printf("Kp: %f.   Ki: %f.    Kd: %f   ",kp,ki,kd);
  Serial.print("Sp:");
 // Serial.print(shooting_angle);
  Serial.print("     Ap:");
  Serial.println(int((ap * 360.0 / shooter_rotation_cpr) / 6.25));
  // Serial.println(enc1.read());
  setPosition(int(speed_m));
    if(track)
    {  digitalWrite(13,LOW);
      track=false;
    }
    else
      {
        digitalWrite(13,HIGH);
        track=true;
      }
}
//int lastTime = 0;


void loop() {

  // //drive(5000);

  // if (millis() - lastTime > 10) {
  //   noInterrupts();
  //   Serial.println(millis() - lastTime);
  //   if (Serial.available() > 0) {
  //     String input = Serial.readString();  //030
  //                                          // Serial.println(input.length());
  //     if (input.length() <= 4) {
  //       //char sign=(input.substring(0, 1)).charAt(0);

  //       shooting_angle = input.toInt();

  //       Serial.write(shooting_angle);
  //       sp_angle = int(shooting_angle * 6.25);
  //     }
  //   }
  //   digitalWrite(13,LOW);
  //   delay(10);
  //   digitalWrite(13,HIGH);
  //   delay(10);
  //   interrupts();
  //   lastTime = millis();
  // }


  //ps4_input();
}
#include <Servo.h>
//#include <cmath>

Servo sr;
Servo sl;
unsigned long previousMillis = 0;
int startPos, endPos, steps = 100;
unsigned long duration;
int currentStep = 0;
bool isMoving = false;

float easeInOutCirc(float x) {
return x < 0.5 ? (1 - sqrt(1 - pow(2 * x, 2))) / 2 : (sqrt(1 - pow(-2 * x + 2, 2)) + 1) / 2;
}

void servomotion(int Spos, int Epos, int time) {
  startPos = Spos;
  endPos = Epos;
  duration = time;
  previousMillis = millis();  // Reset time
  currentStep = 0;
  isMoving = true;  // Start motion
}

void updateServo() {
  if (!isMoving) return;

  unsigned long currentMillis = millis();
  unsigned long stepDuration = duration / steps;

  if (currentMillis - previousMillis >= stepDuration) {
    previousMillis = currentMillis;

    float progress = (float)currentStep / steps;
    float easedProgress = easeInOutCirc(progress);

    int servoPosL = startPos + round((endPos - startPos) * easedProgress);
    int servoPosR = 176 - (startPos + round((endPos - startPos) * easedProgress));

    sr.write(servoPosR);

    sl.write(servoPosL);

    Serial.print("sr: "); Serial.print(servoPosL);
    Serial.print(" | sl: "); Serial.println(servoPosR);

    currentStep++;
    if (currentStep >= steps) {
      isMoving = false;
      currentStep = 0;
    }
  }
}


void setup() {
  Serial.begin(9600);
  sr.attach(9);
  sl.attach(36);
  sr.write(180);
  sl.write(0);
  Serial.println("...starting");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    int Spos, Epos, dur;
    
    // Allow both formats: "09018005" OR "90 180 5"
    if (sscanf(input.c_str(), "%d %d %d", &Spos, &Epos, &dur) == 3) {
        servomotion(Spos, Epos, dur * 1000);
    } else {
        Serial.println("Invalid input format! Use: SSS EEE tt (e.g., 90 180 5)");
    }
}


  updateServo();  // Continuously update servo position
}
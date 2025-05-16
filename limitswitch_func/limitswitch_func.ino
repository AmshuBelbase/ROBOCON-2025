// bool limitSwitchState[3][2] = {{0,1} , {2,3}, {4,5}};
int limitSwitch[3][2] = {{41, 40} , {2,3}, {4,5}};     //down, up
int pin[3][2] = {{24,12} , {2,3}, {4,5}};  // {{pwm},{dir}}
bool flag[2] = {false};

void setup() {
  
  for(int i=0;i<3;i++){
    for (int j = 0; j < 2; j++){
      pinMode(limitSwitch[i][j], INPUT_PULLUP);
    }
  }

  for(int i=0;i<3;i++){
    for (int j = 0; j < 2; j++){
      pinMode(pin[i][j], OUTPUT);
    }
    digitalWrite(pin[i][1], LOW);
    analogWrite(pin[i][0], 0);
  }

}

int val = 0;

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    val=input.toInt();
    control(val);         
  }
}

void control(int num){
  if (flag[num] == 0){
    Serial.println("UP");
    while (digitalRead(limitSwitch[num][1]) != LOW){
      digitalWrite(pin[num][1], HIGH);
      analogWrite(pin[num][0], 255*64);
      // limitSwitchState[num][1] = digitalRead(limitSwitch[num][1]);
    }
    digitalWrite(pin[num][1], LOW);
    analogWrite(pin[num][0], 255*0);
    flag[num] = 1;
  }else{
    Serial.println("DOWN");
    while (digitalRead(limitSwitch[num][0]) != LOW){
      digitalWrite(pin[num][1], LOW);
      analogWrite(pin[num][0], 255*64);
      // limitSwitchState[num][0] = digitalRead(limitSwitch[num][0]);
    }
    digitalWrite(pin[num][1], LOW);
    analogWrite(pin[num][0], 255*0);
    flag[num] = 0;

  }

}


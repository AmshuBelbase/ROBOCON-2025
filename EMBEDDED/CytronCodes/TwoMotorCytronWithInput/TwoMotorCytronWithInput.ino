int dir1 = 8;
int speed1 = 6;

int dir2 = 9;
int speed2=5;

void setup() {
  Serial.begin(9600);
  pinMode(dir1,OUTPUT);
  pinMode(dir2,OUTPUT);
  pinMode(speed1,OUTPUT);
  pinMode(speed2,OUTPUT);

  analogWrite(speed1,0);
  analogWrite(speed2,0);
  digitalWrite(dir1,LOW);
  digitalWrite(dir2,LOW);
}

void loop() {
  if(Serial.available()>0){
    //rpm,delay
    String data = Serial.readString();
    int finalIndex = data.indexOf(',');
    int r = data.substring(0,finalIndex).toInt();
    int del = data.substring(finalIndex+1).toInt();

    int rpm = abs(r);
    int pwm = map(rpm,0,750,0,185);

    Serial.println(r);
    Serial.println(del);
    Serial.println(pwm);

    if(r>0){
      digitalWrite(dir1,LOW);
      digitalWrite(dir2,LOW);

      analogWrite(speed1,pwm);
      analogWrite(speed2,pwm);

      delay(del);

      analogWrite(speed1,0);
      analogWrite(speed2,0);
      
      delay(del);
    }
    else if(r<0){
      digitalWrite(dir1,HIGH);
      digitalWrite(dir2,HIGH);

      analogWrite(speed1,pwm);
      analogWrite(speed2,pwm);
      delay(del);

      analogWrite(speed1,0);
      analogWrite(speed2,0);
      
      delay(del);
    }

    else{
      analogWrite(speed1,0);
      analogWrite(speed2,0);
    }
  }

}
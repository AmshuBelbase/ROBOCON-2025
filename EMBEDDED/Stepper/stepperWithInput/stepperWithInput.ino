int dir=8;
int step=12;
int finalIndex=0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(dir,OUTPUT);
  pinMode(step,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()>0){
    // Input in the form of - 10w100 , 10 revolutions , w direction , 100 speed
    String data = Serial.readString();

    int dirS = data.indexOf('s');
    int dirW = data.indexOf('w');

    int finalIndex=0;

    if(dirS>=0){
      finalIndex = dirS;
    }
    if(dirW>=0){
      finalIndex = dirW;
    }

    char direction = data.charAt(finalIndex);
    int revolutions = data.substring(0,finalIndex).toInt();
    int speed = data.substring(finalIndex+1).toInt();

    Serial.println(revolutions);
    Serial.println(direction);
    Serial.println(speed);

    if(direction == 'w'){
      digitalWrite(dir,LOW);
      for(int i=0;i<revolutions*200;i++){
      digitalWrite(step,LOW);
      delayMicroseconds(speed);
      digitalWrite(step,HIGH);
      delayMicroseconds(speed);
        }
      }

    if(direction=='s'){
      digitalWrite(dir,HIGH);
      for(int i=0;i<revolutions*200;i++){
      digitalWrite(step,LOW);
      delayMicroseconds(speed);
      digitalWrite(step,HIGH);
      delayMicroseconds(speed);
      }
    }
  }
}

int piston[4]={31,32,26,27};
String input = "";

void setup() {
  Serial.begin(9600); 
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);

  for(int i=0;i<4;i++){
    pinMode(piston[i],OUTPUT); 
  }

digitalWrite(piston[2],LOW);
digitalWrite(piston[3],HIGH);
digitalWrite(piston[0],LOW);
digitalWrite(piston[1],HIGH);

delay(500);
digitalWrite(piston[2],LOW);
digitalWrite(piston[3],LOW);
digitalWrite(piston[1],LOW);
digitalWrite(piston[0],LOW);

}

void loop() {
  // if(Serial.available())
  // {
  //   input=Serial.readStringUntil('\n');
  // }

  // if (input == "s"){
  //   Serial.println("ok");
  //   digitalWrite(piston[2],HIGH);
  //   digitalWrite(piston[3],LOW);
  //   delay(500);
    
  //   digitalWrite(piston[2],LOW);
  //   digitalWrite(piston[3],HIGH);
  //   delay(400);
  //   digitalWrite(piston[2],LOW);
  //   digitalWrite(piston[3],LOW);
    
  // }


    for (int i=0; i<4; i++){
      digitalWrite(piston[i],HIGH);
      delay(500);
    }
  
    for (int i=0; i<4; i++){
      digitalWrite(piston[i],LOW);
      delay(500);
    }
  

}

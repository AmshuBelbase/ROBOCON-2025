int channelA=2;
int channelB=3;
int count=0;
int lastValueA=0;
int lastValueB=0;

void spinning(){
  int stateA=digitalRead(channelA);
  int stateB=digitalRead(channelB); 
  
  
  // if((lastValueA==stateA&&stateA==stateB)||(lastValueA!=stateA&&stateA!=stateB)){
  //   count++;
  //   Serial.println(count);
  // }
  // if((lastValueA!=stateA&&stateA==stateB)||(lastValueA==stateA&&stateA!=stateB)){
  //   count--;
  //   Serial.println(count);
  // }


   if(lastValueA==stateA){
    if(stateA==stateB){
      count++;
      Serial.println(count);
    }
    else{
      count--;
      Serial.println(count);
    }
   }
   else{
    if(stateA!=stateB){
      count++;
      Serial.println(count);
    }
    else{
      count--;
      Serial.println(count);
    }
   }

  lastValueA=stateA;
  lastValueB=stateB; 
}
void setup() {
  Serial.begin(9600);
  pinMode(channelA,INPUT_PULLUP);
  pinMode(channelB,INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(channelA),spinning,CHANGE);
  attachInterrupt(digitalPinToInterrupt(channelB),spinning,CHANGE);
}

void loop() {
  

}

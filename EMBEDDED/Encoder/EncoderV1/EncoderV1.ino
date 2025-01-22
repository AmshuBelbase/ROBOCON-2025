int channelA=2;
int channelB=3;
int count=0;
int lastValueA=0;
int lastValueB=0;

void spinning(){
  Serial.print("....");
  int stateA=digitalRead(channelA);
  int stateB=digitalRead(channelB); 
  if(lastValueA!=stateA|| lastValueB!=stateB){
  if(stateA!=stateB){
    Serial.println("Cloclwise");
    count++;
  }
  else{
    Serial.println("Anticlockwise");
    count--;
  }
  lastValueA=stateA;
  lastValueB=stateB; 
  }
  Serial.println(count);
}

void setup() {
  Serial.begin(9600);
  pinMode(channelA,INPUT_PULLUP);
  pinMode(channelB,INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(channelA),spinning,CHANGE);
  //attachInterrupt(digitalPinToInterrupt(channelB),spinning,CHANGE);
}

void loop() {
  

}

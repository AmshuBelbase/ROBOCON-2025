//dribbling testing with pistion 
int dir = 0; //direction pin 
int pwm = 0; // pwm pin 

int pflag = 0; // piston flag 

int piston[2][2] = {
  {26, 27},  // Piston A
  {31, 30}   // Piston B
};

void pistons(){
  if(pflag==0){
    //extend pistons if retracted
    for (int i = 0; i <2; i++){
      digitalWrite(piston[i][0], HIGH); 
      digitalWrite(piston[i][1], LOW); 
    }
    pflag =1;
  }
  //retracxt pistons if extended 
  else if(pflag==1){
    for (int i = 0; i <2; i++){
      digitalWrite(piston[i][0], LOW); 
      digitalWrite(piston[i][1], HIGH); 
    }
    pflag=0;
  }
}

void rollers(){
    digitalWrite(dir,HIGH);
    analogWrite(pwm,255); // dribble - rollers downwards
    delay(350); // wait for ball to bounce back 
    digitalWrite(dir,LOW);
    analogWrite(pwm,255); // reverse rooler upwards to catch the ball 
    stop(); // stop rollers completely to hold the ball
}

void stop(){
    analogWrite(pwm,0);
}

void setup() {
  Serial.begin(9600);

  pinMode(dir, OUTPUT);
  pinMode(pwm, OUTPUT);

  for (int i = 0; i < 2; i++) {
    pinMode(piston[i][0], OUTPUT);  
    pinMode(piston[i][1], OUTPUT);  
  }

  Serial.println("p - pistons \nr - rollers");
}

void loop() {
  if (Serial.available()){
    ch = Serial.read();

    if (ch == 'p'){
      pistons();
      Serial.println("Pistons toggled");
    }

    else if (ch == 'r'){
      rollers();
      Serial.println("Dribbling start");
    }

    else if(ch == 's')
    stop();
  }
}

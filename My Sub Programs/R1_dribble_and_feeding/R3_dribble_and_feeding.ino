/*
* 1. ball with frame comes down
* 2. loose the ball up
* 3. dribble
* 4. frame up
* 5. feeding starts
*/ 

int roll_dir=5;
int roll_pwm=2;

int feed_dir=8;
int feed_pwm=7;

int frame_dir=3;
int frame_pwm=4;

int inp;

//frame down and ball release
void frame_down(){
  digitalWrite(frame_dir,LOW);
  analogWrite(frame_pwm,30);
  delay(100);
  analogWrite(frame_pwm,0);
   
}

void setup(){
 pinMode(roll_dir,OUTPUT);
 pinMode(roll_pwm,OUTPUT);
 pinMode(feed_dir,OUTPUT);
 pinMode(feed_pwm,OUTPUT);
 pinMode(frame_dir,OUTPUT);
 pinMode(frame_pwm,OUTPUT);
 
 Serial.begin(9600);

}

void loop(){
  if (Serial.available()){
    inp = Serial.parseInt();
    Serial.print("You Entered: ");
    Serial.println(inp);
  } 
  
  switch(inp)
  {
    
  }
  // frame down
  if(inp==1){
    digitalWrite(frame_dir,LOW);
    analogWrite(frame_pwm,30);
    delay(100);
    analogWrite(frame_pwm,0);
  }

  //ball up
  if(inp==2){
    digitalWrite(roll_dir,LOW);
    analogWrite(roll_pwm,255);
    delay(200)  
  }

  //dribble
  if(inp==3){
    digitalWrite(roll_dir,HIGH);
    analogWrite(roll_pwm,255);
    delay(400);
    digitalWrite(roll_dir,LOW);
    analogWrite(roll_pwm,255);
    delay(1500);
    analogWrite(roll_pwm,0);
  }

  //frame up 
  if(inp==4){
    digitalWrite(frame_dir,HIGH);
    analogWrite(frame_pwm,45);
    delay(750);
    analogWrite(frame_pwm,35);
    delay(850);
    analogWrite(frame_pwm,0);
  }

  //feeding
  if(inp==5){
    digitalWrite(feed_dir,LOW);
    analogWrite(feed_pwm,255);
    delay(2500);     
    analogWrite(feed_pwm,0);
  }



  //pura pura flow mai
  if(inp==5){
   digitalWrite(frame_dir,LOW);//frame going down
    analogWrite(frame_pwm,40);
    delay(300);
    analogWrite(frame_pwm,25);
    delay(800);
    analogWrite(frame_pwm,0);
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
    analogWrite(frame_pwm,45);
    delay(750);
    analogWrite(frame_pwm,40);
    delay(1000);
    analogWrite(frame_pwm,0);
    // frame pura open hone ke phele feeder start
    digitalWrite(feed_dir,LOW);
     analogWrite(feed_pwm,255);
     // frame - stop
    analogWrite(frame_pwm,0);
    
         delay(2500);
     analogWrite(feed_pwm,0);
   inp=-1;

}

}
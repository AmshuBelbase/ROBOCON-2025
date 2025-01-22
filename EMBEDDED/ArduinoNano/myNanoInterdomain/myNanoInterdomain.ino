int dir1 = 5;
int speed1=6;

int dir2=7
int speed2=9;

int x_offset;
int distance;

int threshold_left=320-64
int threshold_right=320+64


void setup() {
  Serial.begin(115200);
  pinMode(dir1,OUTPUT);
  pinMode(dir2,OUTPUT);
  pinMode(speed1,OUTPUT);
  pinMode(speed2,OUTPUT);
}

void loop() {
  int left_motorSpeed;
  int right_motorSpeed;

  int turnSpeed;
  int forwardSpeed=map(distance,40,500,0,255);

  if(x_offset<threshold_left){
    turnSpeed=-map(x_offset,0,255,0,255);
  }
  if(x_offset>threshold_right){
    turnSpeed=map(x_offset,384,640,0,255);
  }

  left_motorSpeed=forwardSpeed-turnSpeed;
  right_motorSpeed=forwardSpeed+turnSpeed;
  
  if(left_motorSpeed<=0||right_motorSpeed<=0){

  }
  else{
    left_motorSpeed=map(left_motorSpeed,0,510,0,255);
  }


  digitalWrite(dir1,HIGH);
  digitalWrite(dir2,HIGH);

  analogWrite(speed1,left_motorSpeed);
  analogWrite(speed2,right_motorspeed);
}
-
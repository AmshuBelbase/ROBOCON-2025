int dir_pin = 5;
int step_pin = 6;

void setup() {
  Serial.begin(9600);

  pinMode(step_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);
  digitalWrite(dir_pin, LOW);

  Serial.println("Enter data: ");
}

void loop() {

  if (Serial.available() > 0) {
  

   String input = Serial.readString();    // 10w100 (10 - Revolutions, w-direction, 100 - speed)

  int speed = input.substring(3).toInt();
  char direction = input[2];
  int rev = input.substring(0,2).toInt();

  Serial.println("Speed: ");
  Serial.println(speed);

  Serial.println("No. of revolutions: ");
  Serial.println(rev);

  if (direction == 'w'){
    Serial.println("Direction: Forward");
  }else if (direction == 's'){
    Serial.println("Direction: Backward");
  }

  for (int i = 0; i < 200*rev; i++){
    if (direction == 'w'){
      digitalWrite(dir_pin, HIGH); //forward dir
      digitalWrite(step_pin, HIGH);
      delay(speed);
      digitalWrite(step_pin, LOW);
      delay(speed);
    } else if (direction == 's'){
      digitalWrite(dir_pin, LOW); //backward dir
      digitalWrite(step_pin, HIGH);
      delay(speed);
      digitalWrite(step_pin, LOW);
      delay(speed);
    } else{
      Serial.println("Invalid Input");
    }

  }
  }
}



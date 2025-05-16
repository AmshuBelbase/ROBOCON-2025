int piston[3][2] = { {26,27} , {31,30} , {28,29} };

/*
    PISTON LAYOUT (Top to Bottom View of Bot):

        ┌──────────────┐
        │  BALL PUSH   │     piston[0][OUT] = 26
        │              │     piston[0][IN]  = 27
        └──────────────┘

        ┌──────────────┐
        │ DRIBBLE ARM  │     piston[1][OUT] = 31
        │              │     piston[1][IN]  = 30
        └──────────────┘

        ┌──────────────┐
        │ BALL CATCH   │     piston[2][OUT] = 28
        │              │     piston[2][IN]  = 29
        └──────────────┘

    NOTE: piston[function][direction]
          direction = 0 → OUT
                      1 → IN
*/


void setup() {
  Serial.begin(9600);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  for (int i = 0; i < 3; i++){
    for (int j = 0; j < 2; j++){
      pinMode(piston[i][j], OUTPUT);
    }
  }

  pistonControl(0, false);
  pistonControl(1, false);
  pistonControl(2, false);
}

void loop() {
  Serial.println("ok");
  pistonControl(1, true);
  delay(1000);
  pistonControl(1, false);
}

void pistonControl(int pistonNum, bool extend){
  digitalWrite(piston[pistonNum][0], extend);
  digitalWrite(piston[pistonNum][1], !extend);
  delay(500);
  digitalWrite(piston[pistonNum][0], LOW);
  digitalWrite(piston[pistonNum][1], LOW);

}

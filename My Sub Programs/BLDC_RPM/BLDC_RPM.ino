#include <VescUart.h>
VescUart UART;

float current = 0; /** The current in amps */
int led = 13;
volatile int feeder_pwm=19;
volatile int feeder_dir=16;

bool limitSwitchState[8] = {false};
int limitSwitch[6]={14,15,41,40,38,39};

void setup() {
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);
  Serial.begin(2000000); 
  Serial8.begin(115200);  
  while (!Serial8) {;}
  UART.setSerialPort(&Serial8);

  analogWriteResolution(14);
  analogWriteFrequency(0, 9000);

  // feeding
  pinMode(feeder_pwm, OUTPUT);
  pinMode(feeder_dir, OUTPUT);

  analogWrite(feeder_pwm, 0);
  digitalWrite(feeder_dir, LOW);

  for(int i=0;i<6;i++)
  {
    pinMode(limitSwitch[i],INPUT_PULLUP);
  }
 
    for(int i=0;i<2;i++)
    {
      limitSwitchState[i] = digitalRead(limitSwitch[i]); 
      
      Serial.print("Limit Switch ");
      Serial.print(i);
      Serial.print(" :");

      Serial.println(limitSwitchState[i]);
    }  

}

int orpm=0;
int nrpm=0;

void loop() {
    if (Serial.available() > 0) {   
      String input=Serial.readStringUntil('\n');
      nrpm=input.toFloat()*7;   
      if(nrpm != 0){
        Serial.println("Feed UP");
      }else{
        Serial.println("Feed Down");
      }              
    }
    if(nrpm != 0){

      while(orpm < nrpm){
        orpm=orpm+((nrpm-orpm)/700)+((nrpm-orpm)%700);
        UART.setRPM(orpm);
        delay(50);
        // Serial.println("+");
        Serial.println((orpm/7));
      }
      while(orpm > nrpm){
        orpm=orpm-((orpm-nrpm)/700)-((orpm-nrpm)%700);
        UART.setRPM(orpm);
        delay(50);
        // Serial.println("-");
        Serial.println((orpm/7));
      } 

      limitSwitchState[1] = digitalRead(limitSwitch[1]);

      // Serial.print("Limit Switch: ");
      // Serial.println(limitSwitchState[1]);

      while (limitSwitchState[1] != LOW){
        UART.setRPM(orpm);
        digitalWrite(feeder_dir, LOW);
        analogWrite(feeder_pwm, 255*64);
        limitSwitchState[1] = digitalRead(limitSwitch[1]);
      }

      // Serial.print("Limit Switch: ");
      // Serial.println(limitSwitchState[1]);

      digitalWrite(feeder_dir, LOW);
      analogWrite(feeder_pwm, 255*0);

    }else{
      orpm = 0; 

      limitSwitchState[0] = digitalRead(limitSwitch[0]);

      // Serial.print("Limit Switch: ");
      // Serial.println(limitSwitchState[0]);

      while (limitSwitchState[0] != LOW){
        digitalWrite(feeder_dir, HIGH);
        analogWrite(feeder_pwm, 255*64);
        limitSwitchState[0] = digitalRead(limitSwitch[0]);
      }
      digitalWrite(feeder_dir, LOW);
      analogWrite(feeder_pwm, 255*0);

      // Serial.print("Limit Switch: ");
      // Serial.println(limitSwitchState[0]);

    } 
    UART.setRPM(orpm); 
  //   if ( UART.getVescValues() ) {
  //     if(UART.data.avgInputCurrent > 12){
  //       // Serial.println("Values: ");
  //       // Serial.println(UART.data.rpm/7);
  //       // Serial.println((UART.data.inpVoltage)/10);
  //       // Serial.println(UART.data.ampHours);
  //       // Serial.println(UART.data.pidPos);
  //       Serial.println(UART.data.avgMotorCurrent);
  //       // Serial.println(UART.data.avgInputCurrent);
  //       // Serial.println(UART.data.tachometer);
  //   }
  // }
  // UART.setBrakeCurrent(current);
}
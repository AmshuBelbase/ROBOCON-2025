#include <Wire.h>     
#include <Adafruit_Sensor.h>        
#include <Adafruit_BNO055.h>           
#include <utility/imumaths.h>        

#define bno_adr  0x40   // Default I2C address of BNO055
#define pca_adr 0x70   // Default I2C address of PCA9685

Adafruit_BNO055 bno(0, bno_adr);
bool bno_present = false ; //if bno present or not 


void setup()
{
  Serial.begin(115200);
  Wire.begin();
  delay(1000);

  Serial.println("cannibg i2c all");

  int count=0;

  for (uint8_t a=1; a<127; a++) 
  {
    Wire.beginTransmission(a);   

    //send communivcation
    if  (Wire.endTransmission()==0) //true if device found 
    {
      Serial.print("Device found at 0x"); 
      
      //to print address in th right way 
      if (a<16)
        Serial.print("0");

      Serial.println(a,HEX);
      count++ ;
      
      if (a == bno_adr)
      {
        Serial.println("bno found");
        bno_present = true;
      }
      else if (a == pca_adr)
        Serial.println("pca found");
    }
  }

  if (count == 0)
    Serial.println("No Device found");
  
  if (bno_present)//will run only of bno is detced
  {
    if(bno.begin())
    {
      bno.setExtCrystalUse(true);
      Serial.println("bno initailised");
    }
    else
    {
      Serial.println("bno failed initialisation");
    }
  }
}

void loop()
{
  if(false)
  {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    Serial.print("heading :");
    Serial.println(euler.x());
    Serial.print("pitch :");
    Serial.println(euler.y());
    Serial.print("roll :");
    Serial.println(euler.z());
  }
}
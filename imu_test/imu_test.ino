#define IN_ONE 12
#define IN_TWO 13

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
  
Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup(void) 
{
  Serial.begin(9600);

  // Set motor pins for direction
  pinMode(IN_ONE, OUTPUT);
  pinMode(IN_TWO, OUTPUT);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);
}

void loop(void) 
{
  /* Get a new sensor event */ 
  sensors_event_t event; 

  bno.getEvent(&event);

  angle = event.orientation.y;

  if(angle > 0){
    digitalWrite(IN_ONE, HIGH);
    digitalWrite(IN_TWO, LOW);
  }
  else if(angle < 0){
    digitalWrite(IN_ONE, LOW);
    digitalWrite(IN_TWO, HIGH);
  }
  else{
    digitalWrite(IN_ONE, LOW);
    digitalWrite(IN_TWO, LOW);
  }
  
  delay(100);
}
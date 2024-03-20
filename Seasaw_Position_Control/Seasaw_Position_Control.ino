#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <PID_v1.h>

// Output PINS
const int potPin = A0;            // Analog pin for potentiometer

// PIN A
const int motorEnablePinA = 10;     // PWM pin to control motor speed (ENA on L298N)
// PIN B
const int motorEnablePinB = 11;     // PWM pin to control motor speed (ENA on L298N)
const int motorDirectionPin1 = 13;  // Input 1 (IN1 on L298N)
const int motorDirectionPin2 = 12;  // Input 2 (IN2 on L298N) 

// IMU Global Variables
double currentAgngle = 0.0;
double maxAngle = 13.0;
double minAngle = -13.0;
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Control Variables
double error = 0.0;
double errorCalc = 0.0;
double kp = 0.01;
double ki = 0.0;
double kd = 50.0;
float integral = 0;
float prev_error = 0;

// Motor Global Variables
void setup() {
  Serial.begin(2000000);

  // Motors
  pinMode(potPin, INPUT);
  pinMode(motorEnablePinA, OUTPUT);
  pinMode(motorEnablePinB, OUTPUT);
  pinMode(motorDirectionPin1, OUTPUT);
  pinMode(motorDirectionPin2, OUTPUT);

  // IMU Sensors
  Serial.println("Orientation Sensor Test"); Serial.println("");
  /* Initialise the sensor */
  if(!bno.begin()){
  /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);

}

void loop() {
  /* Get a new sensor event */ 
  sensors_event_t event; 
  bno.getEvent(&event);
  currentAgngle = event.orientation.y;   // From -20 degrees to 20 degrees.

  // User Input Potentiometer
  int rawAngleRead = analogRead(potPin);
  int desiredAngle = map(rawAngleRead,0,1023,-20,20);

  // Control Loop
  errorCalc =  desiredAngle - currentAgngle;
  error = abs(errorCalc);
  integral = integral + (error*100);
  float derivative = (error-prev_error)/100;
  float output = (kp*error)+(ki*integral)+(kd*derivative);
  
  prev_error = error;
  
  // Motor Speed Mapping
  int motorSpeed = map(abs(output), 0, 10, 120, 255);

  if ( (errorCalc < -1)  ) {
     // Controlling the direction of the mototor
    analogWrite(motorEnablePinA, motorSpeed);
    analogWrite(motorEnablePinB, 50);
    digitalWrite(motorDirectionPin1, LOW);
    digitalWrite(motorDirectionPin2, HIGH);
    Serial.println("First loop");
    
  } else if ( (errorCalc > 1) ){
    // Controlling the direction of the mototor
    analogWrite(motorEnablePinA, 50);
    analogWrite(motorEnablePinB, motorSpeed);
    digitalWrite(motorDirectionPin1, HIGH);
    digitalWrite(motorDirectionPin2, LOW);
    Serial.println("second loop");
  } else {
    analogWrite(motorEnablePinA, 0);
    analogWrite(motorEnablePinB, 0);
    digitalWrite(motorDirectionPin1, LOW);
    digitalWrite(motorDirectionPin2, LOW);
  }

  // Printing on Serial Monitor 
  Serial.print("Current Angle: ");
  Serial.print(currentAgngle);
  
  Serial.print("| Desired Angle: ");
  Serial.print(desiredAngle);

  Serial.print("| Error: ");
  Serial.print(errorCalc);

  Serial.print("| Motore Speed: ");
  Serial.print(motorSpeed);

  Serial.print("| output: ");
  Serial.println(output);

  

}

#define IN_ONE 12
#define IN_TWO 13

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

double input_angle, angle_output;
double set_angle = 0;
double kp = 1.0, ki = 1.0, kd = 1.0;
  
Adafruit_BNO055 bno = Adafruit_BNO055(55);

PID seasaw_pid(&input_angle, &angle_output, &set_angle, kp, ki, kd, DIRECT);

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

  // Initialize PID
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10); 
  
  delay(1000);
    
  bno.setExtCrystalUse(true);
}

float set_point_position(float max_dist, float set_point_dist1, float set_point_dist2){

}
// Only used if we want to manually create the PID controller without the PID library.
float calculate_pulley_pwm(float angle, float desired_angle){
  float err = abs(desired_angle - angle);
  return err;
}

void loop(void) 
{
  /* Get a new sensor event */ 
  sensors_event_t event; 

  bno.getEvent(&event);

  input_angle = event.orientation.y;            // From -30 degrees to 30 degrees.
  set_angle = 20                                // Desired angle.
  
  // Compute the PID based on angle error.
  seasaw_pid.Compute();

  // Set the direction of the motor using the input pins.
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

  int motorSpeed = map(output, -90, 90, 0, 255);
  motorSpeed = constrain(motorSpeed, 0, 255);

  // Apply PWM signal to control motor speed
  analogWrite(motorPin, motorSpeed);  

  // Map PID output to pwm.
  
  delay(100);
}
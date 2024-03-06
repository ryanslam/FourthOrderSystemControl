#define IN_ONE 12
#define IN_TWO 13

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <PID_v1.h>

double input_angle, angle_output;
double set_angle = 0;
double kp = 1.0, ki = 1.0, kd = 1.0;
  
Adafruit_BNO055 bno = Adafruit_BNO055(55);

PID seasaw_pid(&input_angle, &angle_output, &set_angle, kp, ki, kd, DIRECT);

// Ultrasonic sensor pins.
const int set_trig_pin = 9;
const int set_echoPin = 10;

// Motor driver pins.
const int motor_pin = 11;

float set_duration, set_position;

void setup() {
  Serial.begin(9600);

  Serial.print("starting");
  // Set motor pins for direction.
  pinMode(IN_ONE, OUTPUT);
  pinMode(IN_TWO, OUTPUT);

  // Pins for set position sensor.
  pinMode(set_trig_pin, OUTPUT);
  pinMode(set_echoPin, INPUT);

  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!bno.begin()){
  /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  // Initialize PID
  seasaw_pid.SetMode(AUTOMATIC);
  seasaw_pid.SetSampleTime(10); 
  
  delay(1000);
    
  bno.setExtCrystalUse(true);
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
  set_angle = 20;                               // Desired angle.

  // Retrieve distance of set point.
  digitalWrite(set_trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(set_trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(set_trig_pin, LOW);

  set_duration = pulseIn(set_echoPin, HIGH);
  set_position = (set_duration*.0343)/2;
  Serial.print("Distance: ");
  Serial.println(set_position);
  
  // Compute the PID based on angle error.
  seasaw_pid.Compute();

  // Set the direction of the motor using the input pins.
  if(input_angle > 0){
    digitalWrite(IN_ONE, HIGH);
    digitalWrite(IN_TWO, LOW);
  }
  else if(input_angle < 0){
    digitalWrite(IN_ONE, LOW);
    digitalWrite(IN_TWO, HIGH);
  }
  else{
    digitalWrite(IN_ONE, LOW);
    digitalWrite(IN_TWO, LOW);
  }

  // Generate PWM for DC Motors (Based on angle).
  int motor_speed = map(angle_output, -30, 30, 0, 255);
  motor_speed = constrain(motor_speed, 0, 255);

  // Apply PWM signal to control motor speed
  analogWrite(motor_pin, motor_speed);
  
  delay(100);
}
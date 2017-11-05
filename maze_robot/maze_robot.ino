#include<DistanceSensor.h>
#include "DistanceFlightSensors.h"
#include "Driver.h"

#define FRONT_SENSOR_SHUTDOWN_PIN 49
#define LEFT_SENSOR_SHUTDOWN_PIN 53
#define RIGHT_SENSOR_SHUTDOWN_PIN 51
#define TOP_SENSOR_SHUTDOWN_PIN 47

#define MOTOR_A_CH_1 3
#define MOTOR_A_CH_2 4
#define MOTOR_B_CH_1 5
#define MOTOR_B_CH_2 6


// Minimum stopping distance in CM
long STOP_DISTANCE_THRESHOLD = 10;

uint16_t STOP_DISTANCE_FLIGHT_THRESHOLD = 20;


uint16_t frontDistance = 0;
uint16_t leftDistance = 0;
uint16_t rightDistance = 0;
uint16_t topDistance = 0;

Adafruit_VL53L0X FRONT_SENSOR = Adafruit_VL53L0X();
Adafruit_VL53L0X LEFT_SENSOR = Adafruit_VL53L0X();
Adafruit_VL53L0X RIGHT_SENSOR = Adafruit_VL53L0X();
Adafruit_VL53L0X TOP_SENSOR = Adafruit_VL53L0X();
DistanceFlightSensors sensors(&FRONT_SENSOR,&LEFT_SENSOR,&RIGHT_SENSOR,&TOP_SENSOR);


//// Motor A PWM Pins
//int MOTOR_A_CH_1 = 3;
//int MOTOR_A_CH_2 = 4;
//
//// Motor B PWM Pins
//int MOTOR_B_CH_1 = 5;
//int MOTOR_B_CH_2 = 6;

Driver driver (MOTOR_A_CH_1,MOTOR_A_CH_2,MOTOR_B_CH_1,MOTOR_B_CH_2);

// Sensor
DistanceSensor sensor(12, 13);

boolean needToTurn = false;
boolean turningLeft = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(MOTOR_A_CH_1, OUTPUT);
  pinMode(MOTOR_A_CH_2, OUTPUT);
  pinMode(MOTOR_B_CH_1, OUTPUT);
  pinMode(MOTOR_B_CH_2, OUTPUT);

  pinMode(FRONT_SENSOR_SHUTDOWN_PIN,OUTPUT);
  pinMode(LEFT_SENSOR_SHUTDOWN_PIN,OUTPUT);
  pinMode(RIGHT_SENSOR_SHUTDOWN_PIN,OUTPUT);
  pinMode(TOP_SENSOR_SHUTDOWN_PIN,OUTPUT);
  setDistanceSensors();  
}

void loop() {

  topDistance = sensors.getDistanceFromTopSensor();
  Serial.print("Top Sensor "); Serial.print(" Distance (mm): "); Serial.println(topDistance);

  leftDistance = sensors.getDistanceFromLeftSensor();
  Serial.print("Left Sensor "); Serial.print(" Distance (mm): "); Serial.println(leftDistance);

  rightDistance = sensors.getDistanceFromRightSensor();
  Serial.print("Right Sensor "); Serial.print(" Distance (mm): "); Serial.println(rightDistance);

  frontDistance = sensors.getDistanceFromFrontSensor();
  Serial.print("Front Sensor "); Serial.print(" Distance (mm): "); Serial.println(frontDistance);

  delay(2000);

  if(needToTurn){
    if(turningLeft || leftDistance==0 || leftDistance>rightDistance){
      driver.goLeft();
      turningLeft = true;
    }
    else{
      driver.goRight();
      turningLeft = false;
    }
    
  }

  if(frontDistance>=1 && frontDistance<=STOP_DISTANCE_FLIGHT_THRESHOLD){
      driver.halt();
      needToTurn = true;    
  }
  else{
    needToTurn = false;
    turningLeft = false;
    if(frontDistance>255){
        driver.goFoward(255);
    }
    else if(frontDistance<=100){
        driver.goFoward(100);
    }
    else{
      driver.goFoward(frontDistance);
    }
    
  }
}

void setDistanceSensors(){
  digitalWrite(FRONT_SENSOR_SHUTDOWN_PIN,LOW);
  digitalWrite(LEFT_SENSOR_SHUTDOWN_PIN,LOW);
  digitalWrite(RIGHT_SENSOR_SHUTDOWN_PIN,LOW);
  digitalWrite(TOP_SENSOR_SHUTDOWN_PIN,LOW);

  delay(10);

  digitalWrite(FRONT_SENSOR_SHUTDOWN_PIN,HIGH);
  digitalWrite(LEFT_SENSOR_SHUTDOWN_PIN,HIGH);
  digitalWrite(RIGHT_SENSOR_SHUTDOWN_PIN,HIGH);
  digitalWrite(TOP_SENSOR_SHUTDOWN_PIN,HIGH);

  digitalWrite(LEFT_SENSOR_SHUTDOWN_PIN,LOW);
  digitalWrite(RIGHT_SENSOR_SHUTDOWN_PIN,LOW);
  digitalWrite(TOP_SENSOR_SHUTDOWN_PIN,LOW);

  FRONT_SENSOR.begin(0x30);
  
  digitalWrite(LEFT_SENSOR_SHUTDOWN_PIN,HIGH);
  LEFT_SENSOR.begin(0x31);

  digitalWrite(RIGHT_SENSOR_SHUTDOWN_PIN,HIGH);
  RIGHT_SENSOR.begin(0x32);

  digitalWrite(TOP_SENSOR_SHUTDOWN_PIN,HIGH);
  TOP_SENSOR.begin(0x33);
}

//// Forward
//void motorDriveReverse(int speed, int time) {
//  analogWrite(MOTOR_A_CH_1, speed + 12);
//  analogWrite(MOTOR_A_CH_2, 0);
//
//  analogWrite(MOTOR_B_CH_1, speed);
//  analogWrite(MOTOR_B_CH_2, 0);
//
//  delay(time);
//}
//
//// Reverse 
//void motorDriveForward(int speed, int time) {
//  analogWrite(MOTOR_A_CH_1, 0);
//  analogWrite(MOTOR_A_CH_2, speed + 12);
//
//  analogWrite(MOTOR_B_CH_1, 0);
//  analogWrite(MOTOR_B_CH_2, speed);
//
//  delay(time);
//}
//
//// Stop
//void motorDriveStop(int time) {
//  analogWrite(MOTOR_A_CH_1, 0);
//  analogWrite(MOTOR_A_CH_2, 0);
//
//  analogWrite(MOTOR_B_CH_1, 0);
//  analogWrite(MOTOR_B_CH_2, 0);
//
//  delay(time);
//}
//
//// Turn Left
//void motorDriveTurnLeft(int speed, int time) {
//  analogWrite(MOTOR_A_CH_1, speed);
//  analogWrite(MOTOR_A_CH_2, 0);
//
//  analogWrite(MOTOR_B_CH_1, 0);
//  analogWrite(MOTOR_B_CH_2, 0);
//
//  delay(time);
//}
//
//// Turn Right
//void motorDriveTurnRight(int speed, int time) {
//  analogWrite(MOTOR_A_CH_1, 0);
//  analogWrite(MOTOR_A_CH_2, 0);
//
//  analogWrite(MOTOR_B_CH_1, speed);
//  analogWrite(MOTOR_B_CH_2, 0);
//  delay(time);
//}

#include "DistanceFlightSensors.h"

#define FRONT_SENSOR_SHUTDOWN_PIN 49
#define LEFT_SENSOR_SHUTDOWN_PIN 53
#define RIGHT_SENSOR_SHUTDOWN_PIN 51
#define TOP_SENSOR_SHUTDOWN_PIN 47


Adafruit_VL53L0X FRONT_SENSOR = Adafruit_VL53L0X();
Adafruit_VL53L0X LEFT_SENSOR = Adafruit_VL53L0X();
Adafruit_VL53L0X RIGHT_SENSOR = Adafruit_VL53L0X();
Adafruit_VL53L0X TOP_SENSOR = Adafruit_VL53L0X();
DistanceFlightSensors sensors(&FRONT_SENSOR,&LEFT_SENSOR,&RIGHT_SENSOR,&TOP_SENSOR);

void setup() {
  Serial.begin(9600);
  pinMode(FRONT_SENSOR_SHUTDOWN_PIN,OUTPUT);
  pinMode(LEFT_SENSOR_SHUTDOWN_PIN,OUTPUT);
  pinMode(RIGHT_SENSOR_SHUTDOWN_PIN,OUTPUT);
  pinMode(TOP_SENSOR_SHUTDOWN_PIN,OUTPUT);
  setDistanceSensors();  
}


void loop() {
    
  Serial.println("Reading a measurement... ");

  uint16_t distance = sensors.getDistanceFromTopSensor();
  Serial.print("Top Sensor "); Serial.print(" Distance (mm): "); Serial.println(distance);

  distance = sensors.getDistanceFromLeftSensor();
  Serial.print("Left Sensor "); Serial.print(" Distance (mm): "); Serial.println(distance);

  distance = sensors.getDistanceFromRightSensor();
  Serial.print("Right Sensor "); Serial.print(" Distance (mm): "); Serial.println(distance);

  distance = sensors.getDistanceFromFrontSensor();
  Serial.print("Front Sensor "); Serial.print(" Distance (mm): "); Serial.println(distance);

  delay(2000);
  
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




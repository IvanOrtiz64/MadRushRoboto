#include "Driver.h"
#include <Servo.h>

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

uint16_t STOP_DISTANCE_FLIGHT_THRESHOLD = 250;

int reps = 0;
uint16_t frontDistance = 0;
uint16_t leftDistance = 0;
uint16_t rightDistance = 0;
uint16_t topDistance = 0;

Servo myservo;  // create servo object to control a servo
int pos = 0;    // variable used to update the servo position
boolean roofDetected = false;
boolean outOfMaze = false;
boolean gongFound = false;
boolean kill = false;
char input = 'f';

Adafruit_VL53L0X FRONT_SENSOR = Adafruit_VL53L0X();
Adafruit_VL53L0X LEFT_SENSOR = Adafruit_VL53L0X();
Adafruit_VL53L0X RIGHT_SENSOR = Adafruit_VL53L0X();
Adafruit_VL53L0X TOP_SENSOR = Adafruit_VL53L0X();
DistanceSensor ultrasonicSensor(12, 13);
DistanceFlightSensors sensors(&FRONT_SENSOR,&LEFT_SENSOR,&RIGHT_SENSOR,&TOP_SENSOR,&ultrasonicSensor);
Driver driver (MOTOR_A_CH_1,MOTOR_A_CH_2,MOTOR_B_CH_1,MOTOR_B_CH_2,&sensors);
// Sensor


boolean needToTurn = false;
boolean turningLeft = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial3.begin(9600);

  pinMode(MOTOR_A_CH_1, OUTPUT);
  pinMode(MOTOR_A_CH_2, OUTPUT);
  pinMode(MOTOR_B_CH_1, OUTPUT);
  pinMode(MOTOR_B_CH_2, OUTPUT);

  pinMode(FRONT_SENSOR_SHUTDOWN_PIN,OUTPUT);
  pinMode(LEFT_SENSOR_SHUTDOWN_PIN,OUTPUT);
  pinMode(RIGHT_SENSOR_SHUTDOWN_PIN,OUTPUT);
  pinMode(TOP_SENSOR_SHUTDOWN_PIN,OUTPUT);
  setDistanceSensors();  
  initializeDongGonger();
  while(Serial3.read()!= 'b'){
    Serial3.println(Serial3.read());
  }
  Serial3.println("outside of setup while");
  driver.enterMaze();
}

void loop() {
  
  input = Serial3.read();
  if(input == 'x'){
    setKill();
    Serial3.println("kill true");
  }
  if(input == 'r')
  {
    resetKill();
    Serial.println("kill false");
  }
  
  if(input == 't'){
    roofDetected =false;
    initializeDongGonger();
  }
  
  while(!kill){
    topDistance = sensors.getDistanceFromTopSensor();
  Serial.print("Top Sensor "); Serial.print(" Distance (mm): "); Serial.println(topDistance);
  Serial3.print("Top Sensor "); Serial3.print(" Distance (mm): "); Serial3.println(topDistance);

  //delay(5000);

  leftDistance = sensors.getDistanceFromLeftSensor();
  Serial.print("Left Sensor "); Serial.print(" Distance (mm): "); Serial.println(leftDistance);
  Serial3.print("Left Sensor "); Serial3.print(" Distance (mm): "); Serial3.println(leftDistance);
  
  //delay(5000);

  rightDistance = sensors.getDistanceFromRightSensor();
  Serial.print("Right Sensor "); Serial.print(" Distance (mm): "); Serial.println(rightDistance);
  Serial3.print("Right Sensor "); Serial3.print(" Distance (mm): "); Serial3.println(rightDistance);

   //delay(5000);

  frontDistance = sensors.getDistanceFromFrontSensor();
  Serial.print("Front Sensor "); Serial.print(" Distance (mm): "); Serial.println(frontDistance);
  Serial3.print("Front Sensor "); Serial3.print(" Distance (mm): "); Serial3.println(frontDistance);
  
  Serial3.print("Kill variable "); Serial3.println(kill);
  Serial3.print("roof detected variable "); Serial3.println(roofDetected);
  Serial3.print("outOfMaze variable "); Serial3.println(outOfMaze);
  
    if(input == 'x'){
    setKill();
    Serial3.println("kill true");
  }
    if(input == 't'){
    roofDetected =false;
    initializeDongGonger();
  }
    Serial3.println("main while not kill");
  //figure way to pause the loop
  //read/assign all sensors
  //prit all sensors
  while(!roofDetected && !kill){
    Serial3.println("in while !roofdetected");
    int currentCase = driver.goFowardUntilIntersection();
    Serial.println(currentCase);
    Serial3.print("Intersection reached "); Serial3.print(" case number: "); Serial3.println(currentCase);
    delay(250);
    if(currentCase==7){
      roofDetected = true;
      outOfMaze = true;
      driver.exitMaze();
    }
    else if(currentCase==6 || currentCase==5 || currentCase==3 || currentCase==1){
      delay(250);
      driver.goLeft();      
      driver.goFoward();
      delay(300);
      driver.halt();
    }
    else if(currentCase==0){
      delay(250);
      driver.goLeft();
      delay(250);
      driver.goLeft();
    }
    else if(currentCase==2){
      delay(250);
      driver.goRight();      
      driver.goFoward();
      delay(300);
      driver.halt();
    }
    else if(currentCase==4){
      delay(250);
      driver.goFoward();
      delay(300);
      driver.halt();
    }
    if(input == 'x'){
    setKill();
    delay(1000);
    Serial3.println("kill true");
  }
  input = Serial3.read();
  }

  if(!kill && roofDetected){
    Serial3.println("Going to hit the gong");
    reps++;
    destroyGong(reps);
    if(reps == 3)
    {
      reps = -3;
    }
  }
  
  input = Serial3.read();
  }
}

void destroyGong(int reps){
  swingSaidDongGonger();
  delay(250);
  driver.swing(reps);
  delay(100);
  driver.halt();
}

void setKill(){
  kill = true;
}
void resetKill(){
  kill = false;
}

void initializeDongGonger(){
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo.write(170); //staionary position
  delay(750);
}

void swingSaidDongGonger(){
   delay(100);
  for (pos = 140; pos >= 75; pos -= 6) { // goes from 140 degrees to 75 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(10);                      // waits 10ms for the servo to reach the position
    Serial.println("forward position: "); Serial.println(pos); 
  }
  for (pos = 75; pos <= 138; pos +=8) { // goes from 75 degrees to 140 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(150);  
    Serial.println("reverse position: "); Serial.println(pos);// waits 150ms for the servo to reach the position
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

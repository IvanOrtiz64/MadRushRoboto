#include "Driver.h"
#include "Arduino.h"

Driver::Driver(int motorAlphaCh1, int motorAlphaCh2, int motorBetaCh1, int motorBetaCh2, DistanceFlightSensors*sensors){
 
    MOTOR_A_CH_1 = motorAlphaCh1;
    MOTOR_A_CH_2 = motorAlphaCh2;

    MOTOR_B_CH_1 = motorBetaCh1;
    MOTOR_B_CH_2 = motorBetaCh2;

    SENSORS = sensors;
    
    
}

void Driver:: setStopDistance(uint16_t stopDistance){
  STOP_DISTANCE_FLIGHT_THRESHOLD = stopDistance;
}

boolean Driver::isLeftSideIsLonger(){
  uint16_t leftDistance = SENSORS->getDistanceFromLeftSensor();
  uint16_t rightDistance = SENSORS->getDistanceFromRightSensor();

  if(leftDistance==0 && rightDistance!=0){
    return true;
  }
  else if(leftDistance!=0 && rightDistance==0){
    return false;
  }
  else if(leftDistance>(rightDistance-20)){
    return true;
  }
  else if (leftDistance<(rightDistance-20)){
    return false;
  }
  else{
    return true;
  }
  
}

// return 0 = robot can only move back 
// return 1 = robot can do a left turn
// return 2 = robot can do a right turn
// return 3 = robot can do both a left or right turn
// return 4 = robot can do both a right or go straight
// return 5 = robot can do both a left turn, and go straight
// return 6 = robot can do both a left turn, right turn, or can go straight
int Driver:: goFowardUntilIntersection(){
  uint16_t frontDistance = 0;
  uint16_t leftDistance = 0;
  uint16_t rightDistance = 0;
  boolean keepDriving = true;
  boolean ableToTurnLeft = false;
  boolean ableToTurnRight = false;
  boolean ableToMoveFoward = true;
  int currentCase = -1;
  
  while(keepDriving){
    frontDistance = SENSORS->getDistanceFromFrontSensor();
    leftDistance = SENSORS->getDistanceFromLeftSensor();
    rightDistance = SENSORS->getDistanceFromRightSensor();
    
    if(frontDistance>=1 && frontDistance <= STOP_DISTANCE_FLIGHT_THRESHOLD){
      halt();
      keepDriving = false;
      ableToMoveFoward = false;
    }
    
    if(leftDistance==0 || leftDistance > 250){
      halt();
      keepDriving = false;
      ableToTurnLeft = true;
    }
    
    if(rightDistance==0 || rightDistance > 250){
       halt();
       keepDriving = false;
       ableToTurnRight = true;
    }

    if(keepDriving){
      goFoward();
    }
  }

  if(ableToMoveFoward && ableToTurnLeft && ableToTurnRight){
    currentCase = 6;
  }
  else if(ableToMoveFoward && ableToTurnLeft){
    currentCase = 5;
  }
  else if(ableToMoveFoward && ableToTurnRight){
    currentCase = 4;
  }
  else if(ableToTurnLeft && ableToTurnRight){
    currentCase = 3;
  }
  else if(ableToTurnRight){
    currentCase = 2;
  }
  else if(ableToTurnLeft){
    currentCase = 1;
  }
  else{
    currentCase = 0;
  }

  return currentCase;
}

int Driver::calculateDrivingSpeed(){
  uint16_t frontDistance = SENSORS->getDistanceFromFrontSensor();
  
  if(frontDistance>200){
      return 200;
  }
  else if(frontDistance<=50){
      return 50;
  }
  else{
    return frontDistance;
  }
}

void Driver::goFoward(){
  int motorSpeed = calculateDrivingSpeed();
  uint16_t leftDistance = SENSORS->getDistanceFromLeftSensor();
  uint16_t rightDistance = SENSORS->getDistanceFromRightSensor();

  int rightMotorSpeed = 0;
  int leftMotorSpeed = 0;

  if(isLeftSideIsLonger()){
    rightMotorSpeed = motorSpeed;
    leftMotorSpeed = motorSpeed - 5;
  }
  else{
    rightMotorSpeed = motorSpeed - 5;
    leftMotorSpeed = motorSpeed;
  }
  
  analogWrite(MOTOR_A_CH_1, 0);
  analogWrite(MOTOR_A_CH_2, leftMotorSpeed);

  analogWrite(MOTOR_B_CH_1, 0);
  analogWrite(MOTOR_B_CH_2, rightMotorSpeed);
}

void Driver::goBackward(int motorSpeed){
  analogWrite(MOTOR_A_CH_1, motorSpeed);
  analogWrite(MOTOR_A_CH_2, 0);

  analogWrite(MOTOR_B_CH_1, motorSpeed);
  analogWrite(MOTOR_B_CH_2, 0);
}

void Driver::goLeft(){
  analogWrite(MOTOR_A_CH_1, TURNING_SPEED);
  analogWrite(MOTOR_A_CH_2, 0);

  analogWrite(MOTOR_B_CH_1, 0);
  analogWrite(MOTOR_B_CH_2, TURNING_SPEED);
}

void Driver::goRight(){
  analogWrite(MOTOR_A_CH_1, 0);
  analogWrite(MOTOR_A_CH_2, TURNING_SPEED);

  analogWrite(MOTOR_B_CH_1, TURNING_SPEED);
  analogWrite(MOTOR_B_CH_2, 0);
}

void Driver::halt(){
  analogWrite(MOTOR_A_CH_1, 0);
  analogWrite(MOTOR_A_CH_2, 0);

  analogWrite(MOTOR_B_CH_1, 0);
  analogWrite(MOTOR_B_CH_2, 0);
}


#include "Driver.h"
#include "Arduino.h"

Driver::Driver(int motorAlphaCh1, int motorAlphaCh2, int motorBetaCh1, int motorBetaCh2, DistanceFlightSensors*sensors){
 
    MOTOR_A_CH_1 = motorAlphaCh1;
    MOTOR_A_CH_2 = motorAlphaCh2;

    MOTOR_B_CH_1 = motorBetaCh1;
    MOTOR_B_CH_2 = motorBetaCh2;

    SENSORS = sensors;
    
}

void Driver::goFoward(int motorSpeed){
  uint16_t leftDistance = SENSORS->getDistanceFromLeftSensor();
  uint16_t rightDistance = SENSORS->getDistanceFromRightSensor();

  int rightMotorSpeed = 0;
  int leftMotorSpeed = 0;
  
  if(leftDistance>rightDistance){
    rightMotorSpeed = motorSpeed;
    leftMotorSpeed = motorSpeed - 5;
  }
  else if (leftDistance<rightDistance){
    rightMotorSpeed = motorSpeed - 5;
    leftMotorSpeed = motorSpeed;
  }
  else{
    rightMotorSpeed = motorSpeed;
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


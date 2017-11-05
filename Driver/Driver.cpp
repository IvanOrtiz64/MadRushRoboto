#include "Driver.h"
#include "Arduino.h"

Driver::Driver(int motorAlphaCh1, int motorAlphaCh2, int motorBetaCh1, int motorBetaCh2){
 
    MOTOR_A_CH_1 = motorAlphaCh1;
    MOTOR_A_CH_2 = motorAlphaCh2;

    MOTOR_B_CH_1 = motorBetaCh1;
    MOTOR_B_CH_2 = motorBetaCh2;

    TURNING_SPEED = 150;
}

void Driver::goFoward(int motorSpeed){
  analogWrite(MOTOR_A_CH_1, 0);
  analogWrite(MOTOR_A_CH_2, motorSpeed);

  analogWrite(MOTOR_B_CH_1, 0);
  analogWrite(MOTOR_B_CH_2, motorSpeed);
}

void Driver::goBackward(int motorSpeed){
  analogWrite(MOTOR_A_CH_1, motorSpeed);
  analogWrite(MOTOR_A_CH_2, 0);

  analogWrite(MOTOR_B_CH_1, motorSpeed);
  analogWrite(MOTOR_B_CH_2, 0);
}

void Driver::goLeft(){
  analogWrite(MOTOR_A_CH_1, 0);
  analogWrite(MOTOR_A_CH_2, 0);

  analogWrite(MOTOR_B_CH_1, TURNING_SPEED);
  analogWrite(MOTOR_B_CH_2, 0);
}

void Driver::goRight(){
  analogWrite(MOTOR_A_CH_1, TURNING_SPEED);
  analogWrite(MOTOR_A_CH_2, 0);

  analogWrite(MOTOR_B_CH_1, 0);
  analogWrite(MOTOR_B_CH_2, 0);
}

void Driver::halt(){
  analogWrite(MOTOR_A_CH_1, 0);
  analogWrite(MOTOR_A_CH_2, 0);

  analogWrite(MOTOR_B_CH_1, 0);
  analogWrite(MOTOR_B_CH_2, 0);
}


#include "Driver.h"

// Motor A PWM Pins
int MOTOR_A_CH_1 = 3;
int MOTOR_A_CH_2 = 4;

// Motor B PWM Pins
int MOTOR_B_CH_1 = 5;
int MOTOR_B_CH_2 = 6;

Driver driver (MOTOR_A_CH_1,MOTOR_A_CH_2,MOTOR_B_CH_1,MOTOR_B_CH_2);

void setup() {
  pinMode(MOTOR_A_CH_1, OUTPUT);
  pinMode(MOTOR_A_CH_2, OUTPUT);
  pinMode(MOTOR_B_CH_1, OUTPUT);
  pinMode(MOTOR_B_CH_2, OUTPUT);

}

void loop() {
  driver.goFoward(100);
  delay(2000);
  driver.halt();
  driver.goBackward(100);
  delay(2000);
  driver.halt();
  driver.goLeft();
  delay(2000);
  driver.halt();
  driver.goRight();
  delay(2000);
  driver.halt();

}



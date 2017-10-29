#include<DistanceSensor.h>

// Minimum stopping distance in CM
long STOP_DISTANCE_THRESHOLD = 10;

// Max speed
int MAX_SPEED = 200;

// Min Speed
int MIN_SPEED = 125;

// Motor A PWM Pins
int MOTOR_A_CH_1 = 3;
int MOTOR_A_CH_2 = 4;

// Motor B PWM Pins
int MOTOR_B_CH_1 = 5;
int MOTOR_B_CH_2 = 6;

//20 21
//

// Sensor
DistanceSensor sensor(12, 13);

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(9600);

  pinMode(MOTOR_A_CH_1, OUTPUT);
  pinMode(MOTOR_A_CH_2, OUTPUT);
  pinMode(MOTOR_B_CH_1, OUTPUT);
  pinMode(MOTOR_B_CH_2, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

  

  
  Serial.println("Starting execution of the program.");
  long distanceCm = sensor.getDistance(true);

  /*if(distanceCm >= 10 || distanceCm==0){
      motorDriveForward(MIN_SPEED);
    }
    else{
      motorDriveStop();
    }*/

  if(distanceCm >= 1 && distanceCm <= STOP_DISTANCE_THRESHOLD) {
    motorDriveStop(250);
    motorDriveReverse(100,250);
    motorDriveStop(250);
    motorDriveTurnRight(90,1000);
    motorDriveStop(100);
  } 
  else 
  {
    motorDriveForward(MIN_SPEED,100);
  }

  /*motorDriveForward(MAX_SPEED);
  delay(2000);
  motorDriveStop();
  
  motorDriveReverse(MAX_SPEED);
  delay(2000);
  motorDriveStop();

  motorDriveTurnLeft(75);
  delay(2000);
  
  motorDriveTurnRight(75);
  delay(2000);*/

 
  
}

// Forward
void motorDriveReverse(int speed, int time) {
  analogWrite(MOTOR_A_CH_1, speed + 12);
  analogWrite(MOTOR_A_CH_2, 0);

  analogWrite(MOTOR_B_CH_1, speed);
  analogWrite(MOTOR_B_CH_2, 0);

  delay(time);
}

// Reverse 
void motorDriveForward(int speed, int time) {
  analogWrite(MOTOR_A_CH_1, 0);
  analogWrite(MOTOR_A_CH_2, speed + 12);

  analogWrite(MOTOR_B_CH_1, 0);
  analogWrite(MOTOR_B_CH_2, speed);

  delay(time);
}

// Stop
void motorDriveStop(int time) {
  analogWrite(MOTOR_A_CH_1, 0);
  analogWrite(MOTOR_A_CH_2, 0);

  analogWrite(MOTOR_B_CH_1, 0);
  analogWrite(MOTOR_B_CH_2, 0);

  delay(time);
}

// Turn Left
void motorDriveTurnLeft(int speed, int time) {
  analogWrite(MOTOR_A_CH_1, speed);
  analogWrite(MOTOR_A_CH_2, 0);

  analogWrite(MOTOR_B_CH_1, 0);
  analogWrite(MOTOR_B_CH_2, 0);

  delay(time);
}

// Turn Right
void motorDriveTurnRight(int speed, int time) {
  analogWrite(MOTOR_A_CH_1, 0);
  analogWrite(MOTOR_A_CH_2, 0);

  analogWrite(MOTOR_B_CH_1, speed);
  analogWrite(MOTOR_B_CH_2, 0);
  delay(time);
}

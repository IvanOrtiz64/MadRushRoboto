#include<DistanceSensor.h>

// Minimum stopping distance in CM
long STOP_DISTANCE_THRESHOLD = 30;

// Max speed
int MAX_SPEED = 255;

// Min Speed
int MIN_SPEED = 125;

// Motor A PWM Pins
int MOTOR_A_CH_1 = 3;
int MOTOR_A_CH_2 = 9;

// Motor B PWM Pins
int MOTOR_B_CH_1 = 10;
int MOTOR_B_CH_2 = 11;

// Sensor
DistanceSensor sensor(12, 13);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(MOTOR_A_CH_1, OUTPUT);
  pinMode(MOTOR_A_CH_2, OUTPUT);
  pinMode(MOTOR_B_CH_1, OUTPUT);
  pinMode(MOTOR_B_CH_2, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Starting execution of the program.");
  long distanceCm = sensor.getDistance(true);

  if(distanceCm >= 1 && distanceCm <= STOP_DISTANCE_THRESHOLD) {
    motorDriveStop();
    motorDriveTurnLeft(75);
    boolean notClear = true;
    
    while(notClear) {
      distanceCm = sensor.getDistance(true);

      if(distanceCm > STOP_DISTANCE_THRESHOLD) {
        notClear = false;
      }
    }
    
    motorDriveStop();
    motorDriveForward(75);
  } else {
    motorDriveForward(MIN_SPEED);
  }
}

// Forward
void motorDriveForward(int speed) {
  analogWrite(MOTOR_A_CH_1, speed);
  analogWrite(MOTOR_A_CH_2, 0);

  analogWrite(MOTOR_B_CH_1, speed);
  analogWrite(MOTOR_B_CH_2, 0);
}

// Reverse 
void motorDriveReverse(int speed) {
  analogWrite(MOTOR_A_CH_1, 0);
  analogWrite(MOTOR_A_CH_2, speed);

  analogWrite(MOTOR_B_CH_1, 0);
  analogWrite(MOTOR_B_CH_2, speed);
}

// Stop
void motorDriveStop() {
  analogWrite(MOTOR_A_CH_1, 0);
  analogWrite(MOTOR_A_CH_2, 0);

  analogWrite(MOTOR_B_CH_1, 0);
  analogWrite(MOTOR_B_CH_2, 0);
}

// Turn Left
void motorDriveTurnLeft(int speed) {
  analogWrite(MOTOR_A_CH_1, speed);
  analogWrite(MOTOR_A_CH_2, 0);

  analogWrite(MOTOR_B_CH_1, 0);
  analogWrite(MOTOR_B_CH_2, 0);
}

// Turn Right
void motorDriveTurnRight(int speed) {
  analogWrite(MOTOR_A_CH_1, 0);
  analogWrite(MOTOR_A_CH_2, 0);

  analogWrite(MOTOR_B_CH_1, speed);
  analogWrite(MOTOR_B_CH_2, 0);
}

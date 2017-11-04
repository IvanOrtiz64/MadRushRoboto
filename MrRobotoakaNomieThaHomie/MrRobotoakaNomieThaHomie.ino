#include <Servo.h>
#include <Driver.h>

/*
 * Servo variables initializing
 */
Driver driver;
Servo myservo;  // create servo object to control a servo
int pos = 0;    // variable used to update the servo position

/*
/Set up stuff 
/Start command needs to be here to start the 
/loop when in position to start the race
*/
void setup() {
  Serial.begin(115200);
  initializeDongGonger(); //set pin for servo and write to stationary position
}

void loop() {

  //if maze finished and gongDetected
  //{
  swingSaidDongGonger();
  // }
}

void initializeDongGonger(){
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo.write(160); //staionary position
  delay(750);
}

void swingSaidDongGonger(){
   delay(100);
  for (pos = 140; pos >= 75; pos -= 6) { // goes from 140 degrees to 75 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(10);                      // waits 10ms for the servo to reach the position
    Serial.println("forward positoin: "); Serial.println(pos); 
  }
  for (pos = 75; pos <= 140; pos +=8) { // goes from 75 degrees to 140 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(150);  
    Serial.println("reverse positoin: "); Serial.println(pos);// waits 150ms for the servo to reach the position
  }
}

void leftHandRule(){
  /*
   * left hand rule
   * if you can turn left then turn left
   * else go straight
   * else turn right
   */
   if(frontSensorDistance<=10){
      if(leftSensorDistance>10){
        driver.goLeft();
      }else if(frontSensorDistance>10){
        driver.goStraight();
      }
      else{
        driver.goRight();
      }
    }
}


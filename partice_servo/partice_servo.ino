#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void setup() {
 myservo.attach(12);  // attaches the servo on pin 12 to the servo object
 Serial.begin(115200);
}

void loop() {




  
  
 for (pos = 0; pos <= 45; pos += 5) { // goes from 0 degrees to 180 degrees
   // in steps of 4 degree
   myservo.write(pos);  
   Serial.print("foward pos: ");
   Serial.println(pos); // tell servo to go to position in variable 'pos'
   delay(1000);                       // waits 15ms for the servo to reach the position
 }
 
 for (pos = 45; pos >= 0; pos -= 5) { // goes from 180 degrees to 0 degrees
   myservo.write(pos);              // tell servo to go to position in variable 'pos'
   Serial.print("reverese pos: ");
   Serial.println(pos);
   delay(1000);                       // waits 15ms for the servo to reach the position
 }
}

#include "Adafruit_VL53L0X.h"

int distanceShutDownPins[] = {52,53};
Adafruit_VL53L0X distanceSensors[2];
int numberOfDistanceSensors = (sizeof(distanceShutDownPins)/sizeof(int));

void setup() {
  Serial.begin(115200);
  setDistanceSensors(distanceShutDownPins,distanceSensors);
  
}


void loop() {
    
  Serial.println("Reading a measurement... ");
   
  Serial.print("number of sensors ");  Serial.println(numberOfDistanceSensors);
  for(int i=0; i<numberOfDistanceSensors; i++){
    VL53L0X_RangingMeasurementData_t measure;
    distanceSensors[i].rangingTest(&measure, false); // pass in 'true' to get debug data printout!

    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
      Serial.print("Sensor "); Serial.print(i); Serial.print(" Distance (mm): "); Serial.println(measure.RangeMilliMeter);
    } 
    else {
       Serial.print("Sensor "); Serial.print(i); Serial.println(" out of range ");
    }
     delay(100);
  }    
}

//This method will set the pinmode to output and set all sensors
void reSettingAllDistanceSensors(int shutDownPins[]){
  
  for(int i=0; i<numberOfDistanceSensors; i++){
    pinMode(shutDownPins[i],OUTPUT);
    digitalWrite(shutDownPins[i],LOW);
  }
  delay(10);
  for(int i=0; i<numberOfDistanceSensors; i++){
    digitalWrite(shutDownPins[i],HIGH);
  }
}

//Getting the list of sensors ready for setting their address
void placingAllButFirstShutDownPinToLow(int shutDownPins[]){
  for(int i=1; i<numberOfDistanceSensors; i++){
    digitalWrite(shutDownPins[i],LOW);
  }
}

void setDistanceSensors(int shutDownPins[],Adafruit_VL53L0X sensors[]){
  reSettingAllDistanceSensors(shutDownPins);
  placingAllButFirstShutDownPinToLow(shutDownPins);
  
  byte address = 0x30;
  
  for(int i=0; i<numberOfDistanceSensors; i++){
    digitalWrite(shutDownPins[i],HIGH);
    Adafruit_VL53L0X sensor = Adafruit_VL53L0X();
    sensor.begin(address);
    address++;
    sensors[i] = sensor;
  }
}




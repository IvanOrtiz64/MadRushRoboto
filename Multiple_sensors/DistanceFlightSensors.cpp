#include "DistanceFlightSensors.h"

DistanceFlightSensors :: DistanceFlightSensors(Adafruit_VL53L0X *frontSensor, Adafruit_VL53L0X *leftSensor, Adafruit_VL53L0X *rightSensor, Adafruit_VL53L0X *topSensor){
  FRONT_SENSOR = frontSensor;
  LEFT_SENSOR = leftSensor;
  RIGHT_SENSOR = rightSensor;
  TOP_SENSOR = topSensor;
}


uint16_t DistanceFlightSensors::getDistanceFromTopSensor(){
  VL53L0X_RangingMeasurementData_t measure;
  
   TOP_SENSOR -> rangingTest(&measure, false); // pass in 'true' to get debug data printout!

   uint16_t distance = 0;
    
   if (measure.RangeStatus != 4){  // phase failures have incorrect data
         distance = measure.RangeMilliMeter;
   } 
   delay(100);
   return distance;
}

uint16_t DistanceFlightSensors::getDistanceFromLeftSensor(){
  VL53L0X_RangingMeasurementData_t measure;
  
   LEFT_SENSOR -> rangingTest(&measure, false); // pass in 'true' to get debug data printout!

   uint16_t distance = 0;
    
   if (measure.RangeStatus != 4){  // phase failures have incorrect data
         distance = measure.RangeMilliMeter;
   } 
   delay(100);
   return distance;
}

uint16_t DistanceFlightSensors::getDistanceFromRightSensor(){
  VL53L0X_RangingMeasurementData_t measure;
  
  RIGHT_SENSOR -> rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  uint16_t distance = 0;
    
  if (measure.RangeStatus != 4){  // phase failures have incorrect data
         distance = measure.RangeMilliMeter;
  } 
  delay(100);
  return distance;
}

uint16_t DistanceFlightSensors::getDistanceFromFrontSensor(){
  VL53L0X_RangingMeasurementData_t measure;
  
  TOP_SENSOR -> rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  uint16_t distance = 0;
    
  if (measure.RangeStatus != 4){  // phase failures have incorrect data
         distance = measure.RangeMilliMeter;
  } 
  delay(100);
  return distance;
}





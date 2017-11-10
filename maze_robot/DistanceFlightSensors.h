#include "Adafruit_VL53L0X.h"
#include<DistanceSensor.h>
class DistanceFlightSensors{

  private:
    Adafruit_VL53L0X *FRONT_SENSOR;
    Adafruit_VL53L0X *LEFT_SENSOR;
    Adafruit_VL53L0X *RIGHT_SENSOR;
    Adafruit_VL53L0X *TOP_SENSOR;
    DistanceSensor *ULTRASONIC_SENSOR;

  public:
    DistanceFlightSensors(Adafruit_VL53L0X *frontSensor, Adafruit_VL53L0X *leftSensor, Adafruit_VL53L0X *rightSensor, Adafruit_VL53L0X *topSensor, DistanceSensor *ultrasonicSensor);
    uint16_t getDistanceFromTopSensor();
    uint16_t getDistanceFromLeftSensor();
    uint16_t getDistanceFromRightSensor();
    uint16_t getDistanceFromFrontSensor();
    long getDistanceFromUltrasonicSensor();
};

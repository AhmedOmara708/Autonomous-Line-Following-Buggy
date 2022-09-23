#include "Arduino.h"
#include "ReflectanceSensor.h"

//Define the global variables to configure the reflectance sensor

uint8_t SensorCount = 6;                                  // Number of refectance sensors
uint8_t SensorPins[6] = {23, 22, 19, 27, 25, 32};         // Sensor pins
uint32_t Timeout = 2500;                                  // Sensor reflect timeout (us)

ReflectanceSensor sensor;


void setup() 
{
  sensor.SetSensorPins(SensorPins,SensorCount);           // Set the sensor pins
  sensor.SetTimeout(Timeout);                             // Set sensor timeout (us)
 
  //Begin Serial Communication
  Serial.begin(115200);
}

void loop() 
{
  sensor.ReadSensor();                                   // Read one full set of sensor values

  // Cycle to through all current sensor values and print them
  for(int i=0;i<sensor.GetSensorCount();i++)
    Serial.printf("%d ",sensor.GetSensorValues(i));
  Serial.println();
   
  delay(30);
}

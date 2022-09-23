#ifndef Sonar_h
#define Sonar_h
#include "Arduino.h"

class SonarSensor 
{
  uint8_t SensorPin;
  uint32_t SensorValue;
  uint32_t Timeout;

  public:

    SonarSensor(){
   memset(SensorPins, 0, sizeof(SensorPins));
   memset(SensorValues, 0, sizeof(SensorValues));
   Timeout = 2500;		
	}

    void SetSensorPins(uint8_t* sensor_pin){
		memcpy(SensorPins, pins, sizeof(sensor_pins));
	}
	
	
    void SetTimeout(uint32_t Timeout){
		this->Timeout = Timeout;
	}
	
	
    uint32_t GetTimeout(){
		return (Timeout);
	}
	
    void ReadSensor(){
   uint32_t startTime,totalTime;
   int i;
   
   for (i = 0; i < 1; i++)
   {
     SensorValues[i] = Timeout;
     pinMode(SensorPins[i], OUTPUT);
     digitalWrite(SensorPins[i], HIGH);
   }
   delayMicroseconds(10);
   startTime = micros();
   totalTime = 0;

     pinMode(SensorPin, INPUT);
   while (totalTime < Timeout)
   {
     //noInterrupts();
     totalTime = (uint32_t)(micros() - startTime);
  
     //Save the value of the time for each pin

       if ((digitalRead(SensorPin) == LOW) && (totalTime < SensorValue))
       {
         SensorValue = totalTime;
       }
   }
 }		
	
    uint32_t GetSensorValue(uint8_t i){
		return (SensorValue);
	}
		
	void SetServoangle(uint8_t){
		
	}

};

#endif

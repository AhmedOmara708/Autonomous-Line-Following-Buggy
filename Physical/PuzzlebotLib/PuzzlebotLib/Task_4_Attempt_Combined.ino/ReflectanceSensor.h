/**
 * \file ReflectanceSensor.h
 * \author Eduard Codres, Mario Martinez
 * \copyright Manchester Robotics
 * \date March, 2019
 * \brief ReflectanceSensor class header file.
 */

#ifndef ReflectanceSensor_h
#define ReflectanceSensor_h
#include "Arduino.h"

/**
 * \brief Reflectance sensor class.
 */
class ReflectanceSensor 
{
  uint8_t SensorPins[16];                     ///< line sensor structure. Used to hold sensor info.
  uint32_t SensorValues[16];
  uint8_t SensorCount;
  uint32_t Timeout;

  public:

    /**
     * \brief Constructor. Initialises sensor data with default values.
     */
    ReflectanceSensor();

    /**
     * \brief Inititialises the sensor structure
     * \param sensor_pins, array containg the pins for each individual reflectance sensor
     * \param sensor_count the number of individual reflectance sensors present on the robot
     */
    void SetSensorPins(uint8_t* sensor_pins, uint8_t sensor_count);

    /**
     * \brief Sets the timeout set for measuring the return signal
     * \param Timeout value in microseconds
     */
    void SetTimeout(uint32_t Timeout);

    /**
     * \brief Returns the light sensor associated pin
     * \param i the index of the light sensor for the pin
     */
    uint8_t GetSensorPin(uint8_t i);

    /**
     * \brief Returns the number of light sensors present
     */
    uint8_t GetSensorCount();

    /**
     * \brief Returns the timeout set for measuring the return signal
     */
    uint32_t GetTimeout();

    /**
     * \brief Reads and stores all the sensor return signals
     */
    void ReadSensor();

    /**
     * \brief Returns the stored measurements of sensor return signals
     * \param i the index of the sensor measurement
     */
    uint32_t GetSensorValues(uint8_t i);

};

#endif

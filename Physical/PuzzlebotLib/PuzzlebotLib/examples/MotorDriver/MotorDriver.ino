#include "Arduino.h"
#include "MotorDriver.h"

//Define the global variables to configure the motors

//Right Motor Configuration Variables
int motR_pins[3] = {4, 15, 18};     //Define the Motor Pins
int motR_sign = -1;                 //Define the motor rotation sign

//Left Motor configuration variables
int motL_pins[3] = {2, 12, 0};
int motL_sign = 1;

MotorDriver Mr;
MotorDriver Ml;

//Setup
void setup()
{
  //Set up the Motors
  //Setup the Right Motor object
  Mr.SetBaseFreq(5000);                                             //PWM base frequency setup
  Mr.SetSign(motR_sign);                                            //Setup motor sign
  Mr.DriverSetup(motR_pins[0], 0, motR_pins[1], motR_pins[2]);      //Setup motor pins and channel
  Mr.MotorWrite(0);                                                 //Write 0 velocity to the motor when initialising

  //Setup the Left Motor object
  Ml.SetBaseFreq(5000);
  Ml.SetSign(motL_sign);
  Ml.DriverSetup(motL_pins[0], 1, motL_pins[1], motL_pins[2]);
  Ml.MotorWrite(0);
  
  //Begin Serial Communication
  Serial.begin(115200);
}

//Loop
void loop()
{

  Mr.MotorWrite(-0.5);                       //Set Velocity percentage to the Motors (-1 to 1)
  Ml.MotorWrite(0.4);

  delay(30);                                //Delay before next loop iteration
}

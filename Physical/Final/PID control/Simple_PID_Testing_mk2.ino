#include "Arduino.h"
#include "MotorDriver.h"
#include "Encoder.h"

//Define the global variables to configure the motors

//Right Motor Configuration Variables
int motR_pins[3] = {4, 15, 18};     //Define the Motor Pins
int motR_sign = -1;                 //Define the motor rotation sign

//Left Motor configuration variables
int motL_pins[3] = {2, 12, 0};
int motL_sign = 1;

MotorDriver Mr;
MotorDriver Ml;

Encoder encR(34, 36, 1);
Encoder encL(39, 35, -1);

void interruptcallR(){
    encR.tick();
  }
void interruptcallL(){
    encL.tick();
  }

float error_r = 0, prev_error_r, delta_error_r = 0, integ_error_r = 0;
float error_l = 0, prev_error_l, delta_error_l = 0, integ_error_l = 0;
float m_kp = 0.1, m_ki = 0.5, m_kd = 0;
float left_v = 0;
float right_v = 0;
float t_sampling = 10;
unsigned long t_loop_previous = 0;

float speed_r = 5;
float speed_l = 5;

float output_r = 0;
float output_l = 0;

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

  //Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(encR.get_Pin()), interruptcallR, RISING);
  attachInterrupt(digitalPinToInterrupt(encL.get_Pin()), interruptcallL, RISING);
}

//Loop
void loop()
{
  
  unsigned long t_loop = millis();
  
  if (t_loop - t_loop_previous >= t_sampling) {
    // save the last time the if statement was ran
    float dt = (t_loop - t_loop_previous) / 1000.0;
    left_v = encL.rads();
    right_v = encR.rads();
    t_loop_previous = t_loop;

    Serial.printf("Time %f\n", dt);
    prev_error_r = error_r; // Set previous error for right wheeel
    prev_error_l = error_l; // Set previous error for right wheeel

    error_r = speed_r + right_v; // Calculates error for right wheel
    error_l = speed_l - left_v; // Calculates error for left wheel

    //delta_error_r = (error_r - prev_error_r) / dt;  // Calculates delta error for right wheel
    //delta_error_l = (error_l - prev_error_l) / dt;  // Calculates delta error for left wheel

    integ_error_r = integ_error_r + (error_r * dt); // Calculates integral error for right wheel
    integ_error_l = integ_error_l + (error_l * dt); // Calculates integral error for left wheel

    output_r = (m_kp * error_r) + (m_ki * integ_error_r) + (m_kd * delta_error_r);  // Calculates PID signal
    output_l = (m_kp * error_l) + (m_ki * integ_error_l) + (m_kd * delta_error_l);  // Calculates PID signal
  }
  Serial.printf("Right Error: %f\nLeft Error: %f\n", error_r, error_l);
  Serial.printf("Left Output %f, Right Output %f\n", output_r, output_l);
  //Serial.printf("Encoder Right %f, Encoder Left %f\n", right_v , left_v);
  
  //Serial.printf("Error R %f, Delta Error %f", error_r, delta_error_r);
  Mr.MotorWrite(output_r);                       //Set Velocity percentage to the Motors (-1 to 1)
  Ml.MotorWrite(-output_l);
  //delay(30);                                //Delay before next loop iteration
}

#include <Arduino.h>
#include "ReflectanceSensor.h"
#include "MotorDriver.h"
#include "Encoder.h"
#include "Sonar.h"
#include <ESP32Servo.h>
//#include <mbed.h>

////////////////////////////////////////////////////////////
//Define the global variables to configure the motors

//Right Motor Configuration Variables
int motorR_type=2;
int motR_pins[3] = {4, 15, 18};     //Define the Motor Pins
int motR_sign = -1;                 //Define the motor rotation sign
float gear_ratio_r =34;
float wheel_radius_r = 0.05;

//Left Motor configuration variables
int motorL_type=2;
int motL_pins[3] = {2, 12, 0};
int motL_sign = 1;
float gear_ratio_l =34;
float wheel_radius_l = 0.05;

float distance_between_wheels = 0.18;

//Right Encoder configuration variables
float encoderR_ticks = 48;
//int encoderR_sign = -1;
//int encoderR_MeasType = 1;
int encoderR_pinA = 34;
int encoderR_pinB = 36;

//Left Encoder configuration variables
float encoderL_ticks = 48;
//int encoderL_sign = -1;
//int encoderL_MeasType = 1;
int encoderL_pinA = 35;
int encoderL_pinB = 39;

// Reflectance Sensors configuration variables
int ReflectanceSensor[12] = {23,1,21,13,26,33,22,3,19,27,25,32}; 


// Sonar configuration variables
int Sonar_pin = 14;

int Servo_pin = 5;


//Encoder EncL (encoderL_pinA, encoderL_pinB, gear_ratio_l, encoderL_ticks, wheel_radius_l, distance_between_wheels);
MotorDriver Mr;
MotorDriver Ml;
Encoder Enc (encoderR_pinA, encoderR_pinB, encoderL_pinA, encoderL_pinB, gear_ratio_r, encoderR_ticks, wheel_radius_r, distance_between_wheels);
Servo Servo1(Servo_pin);
ReflectanceSensor RefSen
Sonar S1(Sonar_pin)
Timer myinnerTimer
Timer myouterTimer

Setsensorpins(ReflectanceSensor, 12);
Mr.DriverSetup(4,1,15,18);
Ml.DriverSetup(2,1,12,0);

Ml.SetBaseFreq(5000);
Mr.SetBaseFreq(5000);

// Define functions to set up the encoders and motors 
//void setupMotEnc();
//Setup 

/////////////////////////////////////////////////////////////
void setup() {
  // put your setup code here, to run once:

  //setupMotEnc();
  //Begin Serial Communication
  Serial.begin(115200)

    Mr.MotorWrite(3.5);
    Ml.MotorWrite(3.5); 
    
    S1.Read = 0;    
    myinnerTimer.start();
    myouterTimer.start();
    t_sampling_inner = 0.01;
    t_sampling_outer = 0.001;

    t_finish = 180;
    
    uR_previous = 0;
    uL_previous = 0;
    error_right_previous = 0;
    error_left_previous = 0;
    u_reflect_previous = 0;        
    error_reflectance_previous = 0;

    wR_set = 4;
    wL_set = 4;

    desired_wheel_speed = 3.5;
       
      Servo1.writeSensor(0);
}

void loop() {
  // put your main code here, to run repeatedly:
time = myouterTimer.read();
if time < t_finish {
    
   
    dt = myouterTimer.read();
    
    if dt > t_sampling_outet_sampling_outer
        myouterTimer.start();

       a = RefSen.ReadSensor();

       sonar_dist = S1.ReadSensor();
       
       error_reflectance = 0;
       
if((a[0]<= 1000 )&&(a[1]<= 1000 )&&(a[2]<= 1000 )&&(a[3]<= 1000 )&&(a[4]<= 1000 )&&(a[5]<= 1000 )&&(a[6]<= 1000 )&&(a[7]<= 1000 )&&(a[8]<= 1000 )&&(a[9]<= 1000 )&&(a[10]<= 1000 )&&(a[11]>= 2000 )&&(time <= 140)) {
    error_reflectance = 11;       
else if((a[0]<= 1000 )&&(a[1]<= 1000 )&&(a[2]<= 1000 )&&(a[3]<= 1000 )&&(a[4]<= 1000 )&&(a[5]<= 1000 )&&(a[6]<= 1000 )&&(a[7]<= 1000 )&&(a[8]<= 1000 )&&(a[9]<= 1000 )&&(a[10]>= 2000 )&&(a[11]>= 2000 )&&(time <= 140)) 
    error_reflectance = 10;       
else if((a[0]<= 1000 )&&(a[1]<= 1000 )&&(a[2]<= 1000 )&&(a[3]<= 1000 )&&(a[4]<= 1000 )&&(a[5]<= 1000 )&&(a[6]<= 1000 )&&(a[7]<= 1000 )&&(a[8]<= 1000 )&&(a[9]<= 1000 )&&(a[10]>= 2000 )&&(a[11]<= 1000 )&&(time <= 140)) 
    error_reflectance = 9;
else if((a[0]<= 1000 )&&(a[1]<= 1000 )&&(a[2]<= 1000 )&&(a[3]<= 1000 )&&(a[4]<= 1000 )&&(a[5]<= 1000 )&&(a[6]<= 1000 )&&(a[7]<= 1000 )&&(a[8]<= 1000 )&&(a[9]>= 2000 )&&(a[10]>= 2000 )&&(a[11]<= 1000 )&&(time <= 140)) 
    error_reflectance = 8;
else if((a[0]<= 1000 )&&(a[1]<= 1000 )&&(a[2]<= 1000 )&&(a[3]<= 1000 )&&(a[4]<= 1000 )&&(a[5]<= 1000 )&&(a[6]<= 1000 )&&(a[7]<= 1000 )&&(a[8]<= 1000 )&&(a[9]>= 2000 )&&(a[10]<= 1000 )&&(a[11]<= 1000 )&&(time <= 140)) 
    error_reflectance = 7;
else if((a[0]<= 1000 )&&(a[1]<= 1000 )&&(a[2]<= 1000 )&&(a[3]<= 1000 )&&(a[4]<= 1000 )&&(a[5]<= 1000 )&&(a[6]<= 1000 )&&(a[7]<= 1000 )&&(a[8]>= 2000 )&&(a[9]>= 2000 )&&(a[10]<= 1000 )&&(a[11]<= 1000 )&&(time <= 140)) 
    error_reflectance = 6;
else if((a[0]<= 1000 )&&(a[1]<= 1000 )&&(a[2]<= 1000 )&&(a[3]<= 1000 )&&(a[4]<= 1000 )&&(a[5]<= 1000 )&&(a[6]<= 1000 )&&(a[7]<= 1000 )&&(a[8]>= 2000 )&&(a[9]<= 1000 )&&(a[10]<= 1000 )&&(a[11]<= 1000 )&&(time <= 140)) 
    error_reflectance = 5;
else if((a[0]<= 1000 )&&(a[1]<= 1000 )&&(a[2]<= 1000 )&&(a[3]<= 1000 )&&(a[4]<= 1000 )&&(a[5]<= 1000 )&&(a[6]<= 1000 )&&(a[7]>= 2000 )&&(a[8]>= 2000 )&&(a[9]<= 1000 )&&(a[10]<= 1000 )&&(a[11]<= 1000 )&&(time <= 140)) 
    error_reflectance = 4;
else if((a[0]<= 1000 )&&(a[1]<= 1000 )&&(a[2]<= 1000 )&&(a[3]<= 1000 )&&(a[4]<= 1000 )&&(a[5]<= 1000 )&&(a[6]<= 1000 )&&(a[7]>= 2000 )&&(a[8]<= 1000 )&&(a[9]<= 1000 )&&(a[10]<= 1000 )&&(a[11]<= 1000 )&&(time <= 140)) 
    error_reflectance = 3;
else if((a[0]<= 1000 )&&(a[1]<= 1000 )&&(a[2]<= 1000 )&&(a[3]<= 1000 )&&(a[4]<= 1000 )&&(a[5]<= 1000 )&&(a[6]>= 2000 )&&(a[7]>= 2000 )&&(a[8]<= 1000 )&&(a[9]<= 1000 )&&(a[10]<= 1000 )&&(a[11]<= 1000 )&&(time <= 140)) 
    error_reflectance = 2;
else if((a[0]<= 1000 )&&(a[1]<= 1000 )&&(a[2]<= 1000 )&&(a[3]<= 1000 )&&(a[4]<= 1000 )&&(a[5]<= 1000 )&&(a[6]>= 2000 )&&(a[7]<= 1000 )&&(a[8]<= 1000 )&&(a[9]<= 1000 )&&(a[10]<= 1000 )&&(a[11]<= 1000 )&&(time <= 140)) 
    error_reflectance = 1;
else if((a[0]<= 1000 )&&(a[1]<= 1000 )&&(a[2]<= 1000 )&&(a[3]<= 1000 )&&(a[4]<= 1000 )&&(a[5]>= 2000 )&&(a[6]<= 1000 )&&(a[7]<= 1000 )&&(a[8]<= 1000 )&&(a[9]<= 1000 )&&(a[10]<= 1000 )&&(a[11]<= 1000 )&&(time <= 140)) 
    error_reflectance = -1;
else if((a[0]<= 1000 )&&(a[1]<= 1000 )&&(a[2]<= 1000 )&&(a[3]<= 1000 )&&(a[4]>= 2000 )&&(a[5]>= 2000 )&&(a[6]<= 1000 )&&(a[7]<= 1000 )&&(a[8]<= 1000 )&&(a[9]<= 1000 )&&(a[10]<= 1000 )&&(a[11]<= 1000 )&&(time <= 140)) 
    error_reflectance = -2;
else if((a[0]<= 1000 )&&(a[1]<= 1000 )&&(a[2]<= 1000 )&&(a[3]<= 1000 )&&(a[4]>= 2000 )&&(a[5]<= 1000 )&&(a[6]<= 1000 )&&(a[7]<= 1000 )&&(a[8]<= 1000 )&&(a[9]<= 1000 )&&(a[10]<= 1000 )&&(a[11]<= 1000 )&&(time <= 140)) 
    error_reflectance = -3;
else if((a[0]<= 1000 )&&(a[1]<= 1000 )&&(a[2]<= 1000 )&&(a[3]>= 2000 )&&(a[4]>= 2000 )&&(a[5]<= 1000 )&&(a[6]<= 1000 )&&(a[7]<= 1000 )&&(a[8]<= 1000 )&&(a[9]<= 1000 )&&(a[10]<= 1000 )&&(a[11]<= 1000 )&&(time <= 140)) 
    error_reflectance = -4;
else if((a[0]<= 1000 )&&(a[1]<= 1000 )&&(a[2]<= 1000 )&&(a[3]>= 2000 )&&(a[4]<= 1000 )&&(a[5]<= 1000 )&&(a[6]<= 1000 )&&(a[7]<= 1000 )&&(a[8]<= 1000 )&&(a[9]<= 1000 )&&(a[10]<= 1000 )&&(a[11]<= 1000 )&&(time <= 140)) 
    error_reflectance = -5;
else if((a[0]<= 1000 )&&(a[1]<= 1000 )&&(a[2]>= 2000 )&&(a[3]>= 2000 )&&(a[4]<= 1000 )&&(a[5]<= 1000 )&&(a[6]<= 1000 )&&(a[7]<= 1000 )&&(a[8]<= 1000 )&&(a[9]<= 1000 )&&(a[10]<= 1000 )&&(a[11]<= 1000 )&&(time <= 140)) 
    error_reflectance = -6;
else if((a[0]<= 1000 )&&(a[1]<= 1000 )&&(a[2]>= 2000 )&&(a[3]<= 1000 )&&(a[4]<= 1000 )&&(a[5]<= 1000 )&&(a[6]<= 1000 )&&(a[7]<= 1000 )&&(a[8]<= 1000 )&&(a[9]<= 1000 )&&(a[10]<= 1000 )&&(a[11]<= 1000 )&&(time <= 140)) 
    error_reflectance = -7;
else if((a[0]<= 1000 )&&(a[1]>= 2000 )&&(a[2]>= 2000 )&&(a[3]<= 1000 )&&(a[4]<= 1000 )&&(a[5]<= 1000 )&&(a[6]<= 1000 )&&(a[7]<= 1000 )&&(a[8]<= 1000 )&&(a[9]<= 1000 )&&(a[10]<= 1000 )&&(a[11]<= 1000 )&&(time <= 140)) 
    error_reflectance = -8;
elseif((a[0]<= 1000 )&&(a[1]<= 2000 )&&(a[2]<= 1000 )&&(a[3]<= 1000 )&&(a[4]<= 1000 )&&(a[5]<= 1000 )&&(a[6]<= 1000 )&&(a[7]<= 1000 )&&(a[8]<= 1000 )&&(a[9]<= 1000 )&&(a[10]<= 1000 )&&(a[11]<= 1000 )&&(time <= 140)) 
    error_reflectance = -9;
elseif((a[0]>= 2000 )&&(a[1]>= 2000 )&&(a[2]<= 1000 )&&(a[3]<= 1000 )&&(a[4]<= 1000 )&&(a[5]<= 1000 )&&(a[6]<= 1000 )&&(a[7]<= 1000 )&&(a[8]<= 1000 )&&(a[9]<= 1000 )&&(a[10]<= 1000 )&&(a[11]<= 1000 )&&(time <= 140)) 
    error_reflectance = -10;
else if((a[0]>= 2000 )&&(a[1]<= 1000 )&&(a[2]<= 1000 )&&(a[3]<= 1000 )&&(a[4]<= 1000 )&&(a[5]<= 1000 )&&(a[6]<= 1000 )&&(a[7]<= 1000 )&&(a[8]<= 1000 )&&(a[9]<= 1000 )&&(a[10]<= 1000 )&&(a[11]<= 1000 )&&(time <= 140)) 
    error_reflectance = -11;    
else if((a[0]<= 1000 )&&(a[1]<= 1000 )&&(a[2]<= 1000 )&&(a[3]<= 1000 )&&(a[4]<= 1000 )&&(a[5]<= 1000 )&&(a[6]<= 1000 )&&(a[7]<= 1000 )&&(a[8]<= 1000 )&&(a[9]<= 1000 )&&(a[10]<= 1000 )&&(a[11]<= 1000 )&&(time > 140)) 
    my_alg('wR_set') = 0;
    my_alg('wL_set') = 0;
else if((a[0]<= 1000 )&&(a[1]<= 1000 )&&(a[2]<= 1000 )&&(a[3]<= 1000 )&&(a[4]<= 1000 )&&(a[5]<= 1000 )&&(a[6]<= 1000 )&&(a[7]<= 1000 )&&(a[8]<= 1000 )&&(a[9]<= 1000 )&&(a[10]<= 1000 )&&(a[11]<= 1000 )&&(time <= 75)&&(time > 70)) 
    wR_set = 4;
    wL_set = -4;
else if sonar_dist <=1 && time >= 140
    wR_set = 0;
    wL_set = 0;  
}    

kp_reflect = 0.00075;
ki_reflect = 0.005;
kd_reflect = 0.0004;

u_reflect = (kp_reflect*error_reflectance) + (u_reflect_previous + ki_reflect*error_reflectance*t_sampling_outer') + (kd_reflect*((error_reflectance - error_reflectance_previous)/t_sampling_outer')) ;
u_reflect_previous = u_reflect;        
error_reflectance_previous = error_reflectance;

if error_reflectance < 0 {

    wR_set = 4
    wL_set = 4 + u_reflect;
    
}

if error_reflectance > 0 {

    wR_set = 4 - u_reflect;
    wL_set = 4;
    
}
}

    dt = myinnerTimer.read();
    
    if dt > t_sampling_inner
        myinnerTimer.start();
          
        
        kp_right = 0.15;
        ki_right = 5;
        kd_right = 0.001729041;
        
        kp_left = 0.15;
        ki_left = 5;
        kd_left = 0.001729041;        
                
        
        error_right = wR_set - Enc.GetWheelSpeedRight();        
        error_left = wL_set - Enc.GetWheelSpeedLeft();
        
        
        uR = (kp_right*error_right) + (uR_previous + ki_right*error_right*t_sampling_inner) + (kd_right*((error_right - error_right_previous)/t_sampling_inner)) ;
        uR_previous = uR;        
        error_right_previous = error_right;
        
        uL = (kp_left*error_left) + (uL_previous + ki_left*error_left*t_sampling_inner) + (kd_left*((error_left - error_left_previous)/t_sampling_inner)) ;
        uL_previous = uL;        
        error_left_previous = error_left;

        Mr.MotorWrite(uR);
        Ml.MotorWrite(uL);        
        
}

else
     Mr.MotorWrite(0);
     Ml.MotorWrite(0);
    
    Servo1.writeSensor(0);
    
}
}

/*void setupMotEnc()
{
  Mr.SetBaseFreq(5000);
  Mr.SetMotorType(motorR_type);
  Mr.SetSign(motR_sign);

  if (motorR_type == BRUSHLESS_MOTOR)
  {
    Mr.DriverSetup(motR_pins[0],0, motR_pins[1]);
  }
  else if (motorR_type == DC_MOTOR)
  {
    Mr.DriverSetup(motR_pins[0],0, motR_pins[1], motR_pins[2]);
  }

  //EncR.SetTicksPerRev(encoderR_ticks);
  //EncR.SetGearRatio(gear_ratio_r);
  //EncR.SetSign(encoderR_sign);
  //EncR.SetMeasureType(encoderR_MeasType);
  //EncR.SetEncType(motorR_type);
  //EncR.Encoder_setup(encoderR_pins[0], encoderR_pins[1], 'R');

  Mr.MotorWrite(0);

  //Setup the Left Motor object
  Ml.SetBaseFreq(5000);
  Ml.SetMotorType(motorL_type);
  Ml.SetSign(motL_sign);
  if (motorL_type == BRUSHLESS_MOTOR)
  {
    M1.DriverSetup(motL_pins[0], 1, motL_pins[1]);
  }
  else if (motorR_type == DC_MOTOR)
  {
    Mr.DriverSetup(motL_pins[0],1, motL_pins[1], motL_pins[2]);
  }

  //EncL.SetTicksPerRev(encoderL_ticks);
  //EncL.SetGearRatio(gear_ratio_l);
  //EncL.SetSign(encoderL_sign);
  //EncL.SetMeasureType(encoderL_MeasType);
  //EncL.SetEncType(motorL_type);
  //EncL.Encoder_setup(encoderL_pins[0], encoderL_pins[1], 'L');

  Ml.MotorWrite(0);

}*/

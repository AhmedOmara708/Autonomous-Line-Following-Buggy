#include <Arduino.h>
#include <MotorDriver.h>
#include <ReflectanceSensor.h>
#include "Encoder.h"
#include "Sonar.h"

//Define the global variables to configu_re the motors

//Right Motor Configu_ration Variables
int motR_pins[3] = {4, 15, 18};     //Define the Motor Pins
int motR_sign = -1;                 //Define the motor rotation sign

//Left Motor configu_ration variables
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

//Sonar variable initialisation
Sonar sonar(14);
float distance_sonar = sonar.ping();

//Line sensor variable initialisation
ReflectanceSensor sensor;
float error,prev_error,delta_error,integ_error;
float rs_kp = 7.5, rs_ki = 0,rs_kd = 1.75;
//float rs_kp = 8.5, rs_ki = 0,rs_kd = 1.5; speed = 11
uint8_t SensorCount = 7;                                  // Number of refectance sensors
uint8_t SensorPins[7] = {32, 25, 27, 19, 22, 23 ,21};  // Sensor pins 1 3 5 7 9 11 13
uint32_t Timeout = 2500; 
 

//PID variable initialisation
float error_r = 0, prev_error_r, delta_error_r, integ_error_r = 0;
float error_l = 0, prev_error_l, delta_error_l, integ_error_l = 0;
float m_kp = 0.077, m_ki = 0.9, m_kd = 0;

bool integ_error_on_r, integ_error_on_l = true;
float vel_r, wanted_speed_r, u_r, sat_check_r, output_r = 0; 
float vel_l, wanted_speed_l, u_l, sat_check_l, output_l = 0;

float general_speed = 11.5;
float speed_r = general_speed;
float speed_l = general_speed;

bool Is_positive(float value){
  if (value < 0){
    return false;
  } 
  return true;
}

//Turning variable initialisation
float distance_r = 0;
float distance_l = 0;

float turn_distance = 0.285;
bool turn_around = false;

//Time variable initialisation
float t_sampling_sonar = 500;
unsigned long t_loop_sonar_previous = 0;

float t_sampling_outer = 15;
unsigned long t_loop_outer_previous = 0;

float t_stop = 50;
unsigned long t_loop_stop_previous = 0;

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

  sensor.SetSensorPins(SensorPins,SensorCount);           // Set the sensor pins
  sensor.SetTimeout(Timeout);                             // Set sensor timeout (us)
  
  //Begin Serial Communication
  Serial.begin(115200);

  //Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(encR.get_Pin()), interruptcallR, RISING);
  attachInterrupt(digitalPinToInterrupt(encL.get_Pin()), interruptcallL, RISING);
}

//Loop
void loop()
{
  
  unsigned long t_loop_outer = millis();
  unsigned long t_loop_stop = millis();
  
  if (t_loop_outer - t_loop_outer_previous >= t_sampling_outer) {
    // save the last time the if statement was ran
    float dt = (t_loop_outer - t_loop_outer_previous)/1000.0;
    t_loop_outer_previous = t_loop_outer;

    //line_data = //Get the reflectance sensor values
    sensor.ReadSensor();
    prev_error = error;
    error = 0;
    int flag = 0;

    for(int i=0;i<sensor.GetSensorCount();i++){
      if (sensor.GetSensorValues(i) <1000){ // To fine tune try using 800, 900, 1100
        flag = flag + 1;
        error = error + i - 3;  //Change if indexing is different
      }
    }
    
    if (flag != 0) {
      error = error / flag;
      t_loop_stop_previous = t_loop_stop;
    }
    
    delta_error = (error - delta_error) * dt;

    integ_error = integ_error + (error * dt);  
                                                                      // if it tu_rns to the oppiste direction use
    wanted_speed_r = (rs_kp * error - rs_kd * prev_error) + speed_r;  // wanted_speed_r = -(rs_kp * error - rs_kd * prev_error) + speed_r; 
    wanted_speed_l = -(rs_kp * error - rs_kd * prev_error) + speed_l; // wanted_speed_l = (rs_kp * error - rs_kd * prev_error) + speed_r;

    if (distance_sonar > 0.05 && distance_sonar < 0.33) {
      //wanted_speed_r = 0;
      //wanted_speed_l = 0;
      turn_around = true;
    } else if (distance_sonar < 0.35 ) {
      float scale = distance_sonar - 0.1;
      wanted_speed_r = 0;
      wanted_speed_l = 0;
    }

    vel_r = encR.rads();
    vel_l = encL.rads();
    
    Serial.printf("No Turn\n");
 
    if (turn_around){
      distance_r += -vel_r * 0.0505 * dt;
      distance_l += vel_l * 0.0505 * dt;
      Serial.printf("Turn\n");
      Serial.printf("Right dist: %f Left dist: %f \n", distance_r, distance_l);
      if (distance_r < turn_distance && distance_l < turn_distance){
        wanted_speed_r = -5;
        wanted_speed_l = 5;
        t_loop_stop_previous = t_loop_stop;
      } else {
        turn_around = false;
        distance_r = 0;
        distance_l = 0;
        distance_sonar = 2;
      }
    }

    if (((t_loop_stop - t_loop_stop_previous) > t_stop) && turn_around == false) {
      wanted_speed_r = 0;
      wanted_speed_l = 0;
    }
    
    // Speed PID
    error_r = wanted_speed_r + vel_r;
    error_l = wanted_speed_l - vel_l;

    integ_error_r = integ_error_r + (error_r * dt); // Calcu_lates integral error for right wheel
    integ_error_l = integ_error_l + (error_l * dt); // Calcu_lates integral error for left wheel

    // Check if integral is on for right wheel
    if (integ_error_on_r == true) {
      integ_error_r = integ_error_r + (error_r * dt); 
    } else {
      integ_error_r = integ_error_r - (error_r * dt);;
    }

    // Check if integral is on for left wheel
    if (integ_error_on_l == true) {
      integ_error_l = integ_error_l + (error_l * dt); 
    } else {
      integ_error_l = integ_error_l - (error_l * dt);
    }

    u_r = (m_kp * error_r) + (m_ki * integ_error_r);
    u_l = (m_kp * error_l) + (m_ki * integ_error_l);

    // Perform a satu_ration check on the right wheel
    if (u_r > 1) {
      sat_check_r = 1;
    } else if (u_r < -1) {
      sat_check_r = -1;
    } else {
      sat_check_r = u_r;
    }

    // Perform a satu_ration check on the left wheel
    if (u_l > 1) {
      sat_check_l = 1;
    } else if (u_l < -1) {
      sat_check_l = -1;
    } else {
      sat_check_l = u_l;
    }

    // Update the need of the integal component for right wheel
    if (sat_check_r != u_r && Is_positive(u_r) == Is_positive(error_r)) {
         integ_error_on_r = false;
    } else {
         integ_error_on_r = true;
    }

    // Update the need of the integal component for left wheel
    if (sat_check_l != u_l && Is_positive(u_l) == Is_positive(error_l)) {
         integ_error_on_l = false;
    } else {
         integ_error_on_l = true;
    }

    // Set the final values
    output_r = sat_check_r;
    output_l = sat_check_l;

  }

  unsigned long t_loop_sonar = millis();
  
  if ((t_loop_sonar - t_loop_sonar_previous >= t_sampling_sonar) && turn_around == false) {
    // save the last time the if statement was ran
    t_loop_sonar_previous = t_loop_sonar;

    distance_sonar = sonar.ping();
  }

  Serial.printf("Line Sensor Error: %f \n", error);
  Serial.printf("Right output: %f Left output: %f \n", output_r, output_l);  
  
  Mr.MotorWrite(output_r);                       //Set Velocity percentage to the Motors (-1 to 1)
  Ml.MotorWrite(-output_l);

  delay(30);                                //Delay before next loop iteration
}

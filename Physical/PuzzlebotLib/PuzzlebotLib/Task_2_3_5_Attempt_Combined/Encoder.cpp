#include "Arduino.h"
#include "Encoder.h"
#include "mbed/mbed.h"

/**
 * Encoder types
 */
#define SINGLE_PULSE 1
#define DOUBLE_PULSE 2

#define COUNT_PULSE 1
#define MEASURE_PULSE_DURATION 2

    Encoder::Encoder(int pin1a, int pin1b, int pin2a, int pin2b, int GearRatio, int TickNum, float radius, float distance_between_wheels): inputsignal1(pin1a),inputsignal2(pin1b), inputsignal3(pin2a),inputsignal4(pin2b), gears(GearRatio), ticks(TickNum), r(radius), l(distance_between_wheels){
		distance_one_tick = 2*pi/ticks;
		inverse_gears = 1/gears;
		pinMode(Pin1a, INPUT);
		pinMode(Pin1b, INPUT);
		pinMode(Pin2a, INPUT);
		pinMode(Pin2b, INPUT);
		//digitalWrite(pin1a, HIGH);
		//digitalWrite(pin1b, HIGH);
		//digitalWrite(pin2a, HIGH);
		//digitalWrite(pin2b, HIGH);
		current_ticks_1 = inputsignal1;
		current_ticks_2 = inputsignal3;
		time.start();
		dt = 0.03;
		if time < dt {
			tick_rate_1 = (current_ticks_1 - previous_ticks_1)/dt;
			previous_ticks_1 = current_ticks_1;
			time = 0;
		}
		if time < dt {
			tick_rate_2 = (current_ticks_2 - previous_ticks_2)/dt;
			previous_ticks_2 = current_ticks_2;
			time = 0;
		}		
		wR = distance_one_tick*tick_rate_1*inverse_gears; 
		wL = distance_one_tick*tick_rate_2*inverse_gears;
		vR = wR*r;
		vL = wL*r;
		w = ((wR-wL)/l)*r;
		v = (vR+vL)/2;
	}

	float Encoder::GetWheelSpeedRight {
		Return wR;
	}
	
	float Encoder::GetWheelSpeedLeft {
		Return wL;
	}

	float Encoder::GetWheelSpeedLinearRight {
		Return vR;
	}

	float Encoder::GetWheelSpeedLinearLeft {
		Return vL;
		{

	float Encoder::Total_Angular_speed {
		Return w;
		{	
	
	float Encoder::Total_Linear_speed {
		Return v;
		{

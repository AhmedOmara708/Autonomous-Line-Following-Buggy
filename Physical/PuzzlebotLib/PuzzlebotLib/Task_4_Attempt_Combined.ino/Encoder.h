#ifndef MotorDriver_h
#define MotorDriver_h
#include "Arduino.h"
#include "mbed.h"

/**
 * Encoder types
 */
#define SINGLE_PULSE 1
#define DOUBLE_PULSE 2

#define COUNT_PULSE 1
#define MEASURE_PULSE_DURATION 2

class Encoder
{
private:
    int PinA;
    int PinB;
    int type;
    int sign;
	DigitalIn inputsignal1, inputsignal2;
	int gears, ticks;
	Timer time;
	Timeout myTimeout;
	Ticker myTicker;
	float dt, r, l;
	int current_ticks;
	
        
public:

    Encoder(PinName pin1, PinName pin2, int GearRatio, int TickNum, float radius, float distance_between_wheels): inputsignal1(pin1),inputsignal2(pin2), gears(GearRatio), ticks(TickNum), r(radius), l(distance_between_wheels){
		distance_one_tick = 2*pi/ticks;
		inverse_gears = 1/gears;
		current_ticks_1 = inputsignal1;
		current_ticks_2 = inputsignal2;
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

	float GetWheelSpeedRight (
		Return wR;
	)
	
	float GetWheelSpeedLeft (
		Return wL;
	)

	float GetWheelSpeedLinearRight (
		Return vR;
	)

	float GetWheelSpeedLinearLeft (
		Return vL;
	)

	float Total_Angular_speed (
		Return w;
	)	
	
	float Total_Linear_speed (
		Return v;
	)	
	
}
    
#endif

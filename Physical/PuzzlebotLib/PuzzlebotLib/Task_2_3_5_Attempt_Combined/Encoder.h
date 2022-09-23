#ifndef MotorDriver_h
#define MotorDriver_h
#include "Arduino.h"
//#include "mbed.h"

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
	int inputsignal1, inputsignal2, inputsignal3, inputsignal4;
	int gears, ticks;
	Timer time;
	Timeout myTimeout;
	Ticker myTicker;
	float dt, r, l;
	int current_ticks;
	
        
public:

    Encoder(int pin1a, int pin1b, int pin2a, int pin2b, int GearRatio, int TickNum, float radius, float distance_between_wheels): inputsignal1(pin1a),inputsignal2(pin1b), inputsignal3(pin2a),inputsignal4(pin2b), gears(GearRatio), ticks(TickNum), r(radius), l(distance_between_wheels);

	float GetWheelSpeedRight ();
	
	float GetWheelSpeedLeft ();

	float GetWheelSpeedLinearRight ();

	float GetWheelSpeedLinearLeft ();

	float Total_Angular_speed ();
	
	float Total_Linear_speed ();
	
}
    
#endif

#ifndef Sonar_h
#define Sonar_h
#include "Arduino.h"

class Sonar
{
private:
	int pin;
	double microsec;
	double meters;
	
public:
	Sonar(int PinNum);		
	// Constructor
		
	double ping();
	// getting the distance from the sonar to the object
};

#endif
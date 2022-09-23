#ifndef MotorDriver_h
#define MotorDriver_h
#include "Arduino.h"

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
        
public:

    Encoder(){
		//this->type = DC_MOTOR;
		sign = 1;		
	}
		void EncoderSetup(int PinA, int PinB = 0){
	   this->PinA = PinA;
	   this->PinB = PinB;
			 
	   pinMode(PinA, OUTPUT);
	   pinMode(PinB, OUTPUT);
				  
	   digitalWrite(PinA, LOW);
	   digitalWrite(PinB, LOW);
	  
	   delay(10);		
	}
	
    void SetEncType(int type1){
		//this->type = type;
	}
	
    void SetMesType(int type2){
		//this->type = type;
	}
	
    void SetGearRatio(int ratio){
		
	}
	
	void SetSign(int sign1);
    void SetTickNumber(int ticks1);	
};
    
#endif

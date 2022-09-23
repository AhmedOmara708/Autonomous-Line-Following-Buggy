#include "Sonar.h"


Sonar::Sonar(int PinNum){
pin = PinNum; 
}
	
double Sonar::ping(){
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    delayMicroseconds(2);
    digitalWrite(pin, HIGH);
    delayMicroseconds(5);
    digitalWrite(pin, LOW);

    pinMode(pin, INPUT);
    microsec = pulseIn(pin, HIGH);
    meters = microsec / 1000000.0 *340.0 / 2.0;
    return meters;
  
}

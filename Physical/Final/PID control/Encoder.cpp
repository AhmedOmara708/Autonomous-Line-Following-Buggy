#include "Encoder.h"

Encoder::Encoder(int A, int B, int s){
    PinA = A;
    PinB = B;
    ticks = 0;
    sign = s;
    pinMode(PinA,INPUT);
    pinMode(PinB,INPUT);
	sample_t = millis(); 
}
  
void Encoder::tick(){
  if (digitalRead(PinB) != digitalRead(PinA)) { 
    ticks++;
    } else {
    ticks--; 
  }
}
  
float Encoder::rads(){
	sample_dt = millis() - sample_t;
	float radians = (sign * (ticks / 48.0) * 2.0 * 3.14 * 1000) / (sample_dt);
	reset_ticks();
	sample_t = millis();
    return radians;
}

void Encoder::reset_ticks(){
    ticks = 0;
}
  
int Encoder::get_Pin(){
    return PinA;
}

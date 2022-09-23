#ifndef Encoder_h
#define Encoder_h
#include "Arduino.h"

class Encoder
{
private:
  int PinA, PinB;
  int sign;
  int ticks;
  float sample_t, sample_dt;
  
public:
  Encoder(int A, int B, int s);
  
  void tick();
  
  float rads();

  void reset_ticks();
  
  int get_Pin();
};

#endif
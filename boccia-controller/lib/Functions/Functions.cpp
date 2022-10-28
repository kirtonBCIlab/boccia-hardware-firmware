#include <Arduino.h>

void waitMillis(unsigned long wait_msec)
{
    unsigned long deltaT, t0, t1 = 0;
    t0 = millis();
    
    do
    {
      t1 = millis();
      deltaT = t1-t0;
    }while(deltaT < wait_msec);
}

int signum(float a, float b)
{
  int c = (b < a) - (a < b);
  return c;
}
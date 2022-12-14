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

bool digitalReadDebounce(int pin, unsigned long msec_debounce)
{
  // read the state of the switch into a local variable:
  int r0 = digitalRead(pin);    // Initial reading
  int r1;                       // Updated reading
  unsigned long t0 = millis();  // Set start time
  unsigned long t1;             // Updated time

  // Initial reading
  r0 = digitalRead(pin);
  t0 = millis();

  // Set a msec_debounce window, if reading changes before the window ends, reset window
  do
  {
    // Updated reading
    r1 = digitalRead(pin);
    t1 = millis();

    // If value changed, restart time window
    if (r1 != r0)
    {
      r0 = r1;
      t0 = millis();
    }

  } while ((t1-t0) < msec_debounce);
  
  return r0;
}
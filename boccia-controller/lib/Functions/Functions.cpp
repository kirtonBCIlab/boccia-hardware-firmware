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

bool digitalReadDebounce(int pin, unsigned long msec_debounce, bool is_rising)
{
  // read the state of the switch into a local variable:
  int r0 = digitalRead(pin);    // Initial reading
  int r1;                       // Updated reading
  unsigned long t0 = millis();  // Set start time
  unsigned long t1;             // Updated time

  // Set a msec_debounce window, if reading changes before the window ends, exit and return false
  do
  {
    // Take updated readings
    r1 = digitalRead(pin);
    t1 = millis();

    // Check if value has changed 
    if (is_rising && (r1 == 0)) { return false; }
    else if (!is_rising && (r1 == 1)) { return false; }

  } while ((t1-t0) < msec_debounce);
  
  return true;
}
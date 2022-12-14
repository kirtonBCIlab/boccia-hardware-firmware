#ifndef FUNCTIONS_H
#define FUNCTIONS_H
#include <Arduino.h>

/// @brief Waits the selected time in milliseconds without stopping the code
void waitMillis(unsigned long wait_msec); 

void decodeCommand(long command);

/// @brief Returns signum of a-b
/// @param a Number to compare
/// @param b Reference number
int signum(float a, float b);   // Returns the signum of a-b

/// @brief Returns the boolean from reading a digital pin allowing for set milliseconds to debounce
/// @param pin I/O pin to take the reading from
/// @param msec_debounce Time to allow for debouncing [msec]
bool digitalReadDebounce(int pin, unsigned long msec_debounce);

#endif
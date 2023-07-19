#ifndef FUNCTIONS_H
#define FUNCTIONS_H
#include <Arduino.h>

/// @brief Waits the selected time in milliseconds without stopping the code
/// @param wait_msec Time to wait [msec]
void waitMillis(unsigned long wait_msec); 

// TODO: Move decode command here instead of main
void decodeCommand(long command);

/// @brief Returns signum of a-b
/// @param a Number to compare
/// @param b Reference number
/// @returns 1 if a>b; -1 if a<b; 0 if a==b
int signum(float a, float b);

/// @brief Returns the boolean from reading a digital pin allowing for set milliseconds to debounce.
/// @param pin I/O pin to take the reading from
/// @param msec_debounce Time to allow for debouncing [msec]
/// @param is_rising Boolean to determine if the decoder is used for a rising edge. 0 = falling edge
/// @returns True when the pin has stabilized, false otherwise.
bool digitalReadDebounce(int pin, unsigned long msec_debounce, bool is_rising);

/// @brief Returns the boolean from reading an analog pin, it assumes the pin is active if the value
///        is over the threshold limit
/// @param pin 
/// @param threshold 
/// @param msec_debounce 
/// @param is_rising 
/// @return True when the pin has stabilized, false otherwise.
bool analogReadDebounce(int pin, int threshold, long msec_debounce, bool is_rising);

// bool analogDebounce(int pin, int threshold, long msec_debounce, bool is_rising);

/// @brief Returns true from reading if analog pin is above threshold.
///        false otherwise
/// @param pin Pin to convert from analog to digital reading
/// @param threshold Threshold value to compare pin to [0-255]
bool analogToDigital(int pin, int threshold);

#endif
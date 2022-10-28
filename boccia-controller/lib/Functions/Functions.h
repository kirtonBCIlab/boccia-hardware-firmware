#ifndef FUNCTIONS_H
#define FUNCTIONS_H
#include <Arduino.h>
#include <LinearStepper.h>
#include <AccelStepper.h>

void waitMillis(unsigned long wait_msec); 
void decodeCommand(long command);

/// @brief Returns signum of a-b
/// @param a Number to compare
/// @param b Reference number
int signum(float a, float b);   // Returns the signum of a-b

#endif
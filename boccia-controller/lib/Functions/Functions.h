#ifndef FUNCTIONS_H
#define FUNCTIONS_H
#include <Arduino.h>
#include <LinearStepper.h>
#include <AccelStepper.h>

void waitMillis(unsigned long wait_msec);
void decodeCommand(long command);

#endif
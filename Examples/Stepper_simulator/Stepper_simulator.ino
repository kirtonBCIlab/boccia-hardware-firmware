/*
  Stepper motor simulator
  - This script simulates driving a stepper motor with the A4988 driver using 2 LED
  - One LED is used for the direction, the other is used for the amount of steps
  - The sketch uses the AccelStepper library
  - See tutorial in: https://lastminuteengineers.com/a4988-stepper-motor-driver-arduino-tutorial/
  - The script uses Serial communication to instruct the Arduino the direction and duration of the stepper activation:
  -- F300 = clockwise (forward) activation for 300 msec
  -- R200 = counter clockwise (reverse) activation for 200 msec

*/

// Libraries
#include <AccelStepper.h>

// Define pin connections
const int dirPin = 4;   // Direction pin
const int stepPin = 5;  // Step pin - Must be PWM

#define motorInterfaceType 1  // 1 = Driver

// Create an instance of the stepper motor object
AccelStepper nema8(motorInterfaceType, stepPin, dirPin);

void setup() {
  // Motor setup
  nema8.setMaxSpeed(1000);    // Max speed based [steps/sec]. NEMA8 = 200 steps per revolution
	nema8.setSpeed(100);
	nema8.moveTo(100);

  // Serial connection setup
  Serial.begin(9600); // Start serial connection, select baud rate
}

void loop() {
  if (Serial.available()>0)
  {
    String command =  Serial.read();
    dir = command[0];
    dur = command[1:end];
  }

  	// Change direction once the motor reaches target position
	if (nema8.distanceToGo() == 0) 
		nema8.moveTo(-nema8.currentPosition());

	// Move the motor one step
	nema8.run();

}

int getSpeed(String speed)
{
  
}
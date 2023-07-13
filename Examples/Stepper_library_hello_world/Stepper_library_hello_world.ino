/*
 * Stepper Library Hello World
 * - This is a test script that uses the AccelStepper library
 * - You might have to change the pin_dir and pin_step variables
 * - This code rotates the stepper nsteps in one direction, 
 *   stops for time_stop, and then rotates the stepper nsteps in the
 *   opposite direction
 */

// Include the AccelStepper Library
#include <AccelStepper.h>

// Settings
int pin_dir = 6;                // Arduino pin connected to dir input in driver
int pin_step = 5;               // Arduino pin connected to step input in driver
int nsteps = 200;               // Number of steps to move the motor (usually 200 = one full rotation)
unsigned long time_stop = 1000; // Time to stop the motor [msec]

// Define motor interface type and speed
float motor_speed = 200;  // Default speed [steps/sec]
int motor_direction = 1;  // Direction to move motor
                          // 1 = Clockwise
                          // -1 = Anticlockwise

// Creates an instance of the motor
AccelStepper stepper(AccelStepper::DRIVER, pin_step, pin_dir);

void setup() {
  // Initialize pins as outputs
  pinMode(pin_dir, OUTPUT);
  pinMode(pin_dir, OUTPUT);
  
  // Initialize pins at 0 so that the motor doesn't jump 
  digitalWrite(pin_dir, 0);
  digitalWrite(pin_step, 0);
  
  // Set motor initial speed and the acceleration
  stepper.setMaxSpeed(1000);      // Set max speed in [steps/sec]
  stepper.setAcceleration(10);    // Set acceleration in [steps/(sec^2)]

  // Initial move for motor
  stepper.move(nsteps * motor_direction);
}

void loop() {
  // Change direction once the motor reaches target position
  if(stepper.distanceToGo() == 0)
  {
    waitMillis(time_stop);
    motor_direction *= -1;
    stepper.move(nsteps * motor_direction);
  }
  
  // Move the motor one step
  stepper.run();
}

// Wait a certain amount of millisec
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

/*
 * Stepper Library Hello World
 * - This script uses the AccelStepper library
 */

// Include the AccelStepper Library
#include <AccelStepper.h>

// Define pin connections
const int pin_dir = 6;
const int pin_step = 5;

// Define motor interface type and speed
const int motor_interface_type = 1;
const float motor_speed = 200;

// Creates an instance of the mottor
AccelStepper nema8(AccelStepper::DRIVER, pin_step, pin_dir);
//AccelStepper nema8(motor_interface_type, pin_step, pin_dir);

void setup() {
  digitalWrite(pin_dir, 0);
  digitalWrite(pin_step, 0);
  // set the maximum speed, acceleration factor,
  // initial speed and the target position
  nema8.setMaxSpeed(1000);      // Set max speed in steps/sec
  nema8.setAcceleration(50);    // Set acceleration in steps/(sec^2)
  
  Serial.begin(9600);
  Serial.println("Starting position " + String(nema8.currentPosition()));
  nema8.setSpeed(motor_speed);  // Set speed in steps/sec
  nema8.moveTo(200);            // Move motor to desired step, negative are counter clockwise
  Serial.println("Set position: " + String(nema8.targetPosition()));
  
}

void loop() {
  // Change direction once the motor reaches target position
  if(nema8.distanceToGo() == 0)
  {
    //waitMillis(1000);
//    nema8.moveTo(-200);
    nema8.moveTo(-nema8.currentPosition());
    Serial.println("New set position: " + String(nema8.targetPosition()));
    //nema8.setSpeed(motor_speed);      
  }

  // Print current position every multiple of 10
//  if(nema8.currentPosition()%5 == 0)
//    waitMillis(50);
//    Serial.println("Current position " + String(nema8.currentPosition()));
  
  // Move the motor one step
  nema8.run();
}

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

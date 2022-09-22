#include "BocciaStepper.h"
#include <AccelStepper.h>
  
  void BocciaStepper::moveRun(long relative)
    {
      // Set default values before moving
      setSpeed(getDefaultSpeed());
      setAcceleration(getDefaultAccel());
    
      // Set movement and get there
      move(relative);
      runToPosition();

      // If limit detected
      if (limit_flag)
      {
        setAcceleration(default_accel);
        int step_dir = (relative>0) - (relative<0); // Current direction
        move(nreturn*-step_dir);
        runToPosition();
        limit_flag = 0; // Restart flag
      }

    }

  void BocciaStepper::setNoSteps(int steps)
  {
    nsteps = steps;
  }

  void BocciaStepper::setReturnSteps(int steps)
  {
    nreturn = steps;
  }

  void BocciaStepper::setLimits()
  {

  }

  void BocciaStepper::setLowLimit()
  {
    limits[0] = currentPosition();
  }

  void BocciaStepper::setHighLimit()
  {
    limits[1] = currentPosition();
  }

  void BocciaStepper::findRange()
  {
    noInterrupts(); // Disable temporarily for finding range
    moveRun(nsteps);
    
  }

  void BocciaStepper::setInterruptPin(int pin_sensor)
  {
    pin_interrupt = pin_sensor;
    pinMode(pin_interrupt, INPUT);
    Serial.println("Interrupt pin set: " + String(pin_interrupt));  
  }

  int BocciaStepper::getInterruptPin()
  {
    return pin_interrupt;
  }

  void BocciaStepper::setDefaultSpeed(int speed_val)
  {
    default_speed = speed_val;
  }

  int BocciaStepper::getDefaultSpeed()
  {
    return default_speed;
  }

  void BocciaStepper::setDefaultAccel(int accel_val)
  {
    default_accel = accel_val;
  }

  int BocciaStepper::getDefaultAccel()
  {
    return default_accel;
  }

  void BocciaStepper::limitDetected()
  {
    limit_flag = 1;
    setAcceleration(10 * getDefaultAccel());  // Change acceleration to stop quickly
    stop();
  }

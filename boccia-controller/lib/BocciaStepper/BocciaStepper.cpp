#include "BocciaStepper.h"
#include <AccelStepper.h>
  
  void BocciaStepper::moveRun(long relative)
    {
      // Set default values before moving
      setSpeed(getDefaultSpeed());
      setAcceleration(getDefaultAccel());
    
      // If limits exist, make sure requested movement is within limits
      if (limits[0]!=0 && limits[1]!=0)
      {
        long end_position = relative + currentPosition(); 
        if (end_position < limits[0])
          {
            Serial.println("The requested movement will hit the lower limit\nReadjusting value");
            relative = limits[0] - currentPosition(); 
          }
        else if (end_position > limits[1])
          {
            Serial.println("The requested movement will hit the higher limit\nReadjusting value");
            relative = limits[1] - currentPosition();
          }
      }

      // Set movement and get there
      if (relative!=0)
      {
        move(relative);
        runToPosition();
      }
      

      // If limit detected, quickly stop
      if (limit_flag)
      {
        setAcceleration(default_accel);
        int step_dir = (relative>0) - (relative<0); // Current direction
        move(nreturn*-step_dir);
        runToPosition();
        setLimits();
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
    int temp;

    // Set limits for the first time
    if (limits[0]==0 && limits[1]==0)
    { 
      Serial.println("No limits set");
      limits[1] = currentPosition();
      Serial.println("Upper limit = " + String(limits[1]));
    }
    // Set lower limit if higher limit is already set
    else if (limits[0]==0)
    { 
      limits[0] = currentPosition();
      Serial.println("Lower limit = " + String(limits[0]));
    }
    // Update limits
    else
    {
      Serial.println("Limits updated");
      temp = currentPosition();

      // If higher limit was touched
      if (abs(limits[1]-temp) < abs(limits[0]-temp))
      {
        limits[0] += temp-limits[1];
        limits[1] = temp; 
      }

      // Else, lower limit was touched
      else
      {
        limits[1] += temp-limits[0];
        limits[0] = temp;
      }

      Serial.println("Low limit =" + String(limits[0]));
      Serial.println("High limit = " + String(limits[1]));
      }

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

  void BocciaStepper::findRange()
  {
    int starting_pos = currentPosition();
    moveRun(nsteps);
    moveRun(-nsteps);

  }

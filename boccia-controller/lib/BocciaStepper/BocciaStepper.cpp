#include <BocciaStepper.h>
#include <AccelStepper.h>
#include <Functions.h>
  
  BocciaStepper::BocciaStepper(int pin_step, int pin_dir, int interrupt_pins[2], int nsteps, int nsteps_return, int default_speed, int default_accel, bool use_limits):
  AccelStepper(AccelStepper::DRIVER, pin_step, pin_dir)
  {
    _pin_step = pin_step;
    _pin_dir = pin_dir;
    _nsteps = nsteps;
    _nsteps_return = nsteps_return; 
    
    _default_speed = default_speed;
    _default_accel = default_accel;
    
    for (int i=0; i<2; i++) { _interrupt_pins[i] = interrupt_pins[i]; }
  }

  void BocciaStepper::initializePins()
  {
    pinMode(_pin_step, OUTPUT);
    pinMode(_pin_dir, OUTPUT);
    for (int i=0; i<2; i++) 
    {
      if (_interrupt_pins[i] != 0) { pinMode(_interrupt_pins[i], INPUT); }
    }
    setMaxSpeed(1000);  // Recommended max speed
    setCurrentPosition(0);
  }

  void BocciaStepper::moveRun(long relative)
    {
      // Set default values before moving
      setSpeed(_default_speed);
      setAcceleration(_default_accel);
    
      // If limits exist, make sure requested movement is within limits
      // if (use_limits && (limits[0]!=0 && limits[1]!=0))
      // {
      //   long end_position = relative + currentPosition(); 
      //   if (end_position < limits[0])
      //     {
      //       Serial.println("The requested movement will hit the lower limit\nReadjusting value");
      //       relative = limits[0] - currentPosition(); 
      //     }
      //   else if (end_position > limits[1])
      //     {
      //       Serial.println("The requested movement will hit the higher limit\nReadjusting value");
      //       relative = limits[1] - currentPosition();
      //     }
      // }

      Serial.println("Relative: " + String(relative));
      // Set movement and get there
      move(relative);
      do
      {
        run();

        // If limit detected, quickly stop and return motor
        if (_limit_flag)
        {
          Serial.println("Limit flag up");
          if (digitalReadDebounce(active_interrupt_pin,10,1))
          {
            Serial.println("Limit flag stable");
            // Stop motor
            setAcceleration(_default_accel * 10);
            stop();
            runToPosition();
            
            // Return motor _nsteps_return
            int step_dir = (relative>0) - (relative<0); // Current direction
            setAcceleration(_default_accel);
            Serial.println("Returning: " + String(_nsteps_return*-step_dir));
            move(_nsteps_return*-step_dir);
            runToPosition();
            if (use_limits) { setLimits(); }
            _limit_flag = 0; // Restart flag
          }

        }

      } while (distanceToGo() != 0);  
    }

  void BocciaStepper::releaseBall(long relative)
  {
    moveRun(relative);
    moveRun(-relative+2*_nsteps_return);
  }

  void BocciaStepper::setLimits()
  {
    // Set limits for the first time
    if (limits[0]==0 && limits[1]==0)
    { 
      limits[1] = currentPosition();
      Serial.println("No limits set, upper limit = " + String(limits[1]));
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
      int curr_pos = currentPosition();

      // If higher limit was touched
      if (abs(limits[1]-curr_pos) < abs(limits[0]-curr_pos))
      {
        limits[0] += curr_pos-limits[1];
        limits[1] = curr_pos; 
      }

      // Else, lower limit was touched
      else
      {
        limits[1] += curr_pos-limits[0];
        limits[0] = curr_pos;
      }

      Serial.println("limits updated");
      Serial.println("- New upper limit = " + String(limits[1]));
      Serial.println("- New lower limit = " + String(limits[0]));
      }

  }

  void BocciaStepper::limitDetected()
  {
    _limit_flag = 1;    
  }

  void BocciaStepper::findRange()
  {
    // int starting_pos = currentPosition();
    moveRun(_nsteps);
    moveRun(-_nsteps);
  }

  void BocciaStepper::groundInputs()
  {
    digitalWrite(_pin_step, 0);
    digitalWrite(_pin_dir, 0);
  }

  void BocciaStepper::releaseStartPoint()
  {
    moveRun(66);
    limits[0] = limits[1]-100;
    Serial.println("Current pos: " + String(currentPosition()));
    Serial.println("Lower lim: " + String(limits[0]));
  }
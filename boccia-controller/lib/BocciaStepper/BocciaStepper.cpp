#include <BocciaStepper.h>
#include <AccelStepper.h>
  
  BocciaStepper::BocciaStepper(int pin_step, int pin_dir, int interrupt_pins[2], int nsteps, int nsteps_return, int default_speed, int default_accel)
  {
    _pin_step = pin_step;
    _pin_dir = pin_dir;
    _nsteps = nsteps;
    _nsteps_return = nsteps_return;
    _default_speed = default_speed;
    _default_accel = default_accel;
    for (int i=0; i<sizeof(_interrupt_pins); i++)
    {
      _interrupt_pins[i] = interrupt_pins[i];
    }

  }

  void BocciaStepper::moveRun(long relative)
    {
      // Set default values before moving
      setSpeed(_default_speed);
      setAcceleration(_default_accel);
    
      // If limits exist, make sure requested movement is within limits
      if (_limits[0]!=0 && _limits[1]!=0)
      {
        long end_position = relative + currentPosition(); 
        if (end_position < _limits[0])
          {
            Serial.println("The requested movement will hit the lower limit\nReadjusting value");
            relative = _limits[0] - currentPosition(); 
          }
        else if (end_position > _limits[1])
          {
            Serial.println("The requested movement will hit the higher limit\nReadjusting value");
            relative = _limits[1] - currentPosition();
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
        move(_nsteps_return*-step_dir);
        runToPosition();
        setLimits();
        limit_flag = 0; // Restart flag
      }

    }

  void BocciaStepper::setLimits()
  {
    int temp;

    // Set _limits for the first time
    if (_limits[0]==0 && _limits[1]==0)
    { 
      Serial.println("No _limits set");
      _limits[1] = currentPosition();
      Serial.println("Upper limit = " + String(_limits[1]));
    }
    // Set lower limit if higher limit is already set
    else if (_limits[0]==0)
    { 
      _limits[0] = currentPosition();
      Serial.println("Lower limit = " + String(_limits[0]));
    }
    // Update _limits
    else
    {
      Serial.println("_limits updated");
      temp = currentPosition();

      // If higher limit was touched
      if (abs(_limits[1]-temp) < abs(_limits[0]-temp))
      {
        _limits[0] += temp-_limits[1];
        _limits[1] = temp; 
      }

      // Else, lower limit was touched
      else
      {
        _limits[1] += temp-_limits[0];
        _limits[0] = temp;
      }

      Serial.println("Low limit =" + String(_limits[0]));
      Serial.println("High limit = " + String(_limits[1]));
      }

  }

  void BocciaStepper::limitDetected()
  {
    limit_flag = 1;
    setAcceleration(10 * _default_accel);  // Change acceleration to stop quickly
    stop();
  }

  void BocciaStepper::findRange()
  {
    // int starting_pos = currentPosition();
    moveRun(_nsteps);
    moveRun(-_nsteps);

  }

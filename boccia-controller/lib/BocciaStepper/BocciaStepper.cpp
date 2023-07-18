#include <BocciaStepper.h>
#include <AccelStepper.h>
  
  BocciaStepper::BocciaStepper(int pin_step, int pin_dir, int interrupt_pins[2], int nsteps, int nsteps_return, int default_speed, int default_accel):
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
      move(relative);
      do
      {
        run();

        // If limit detected, quickly stop and return motor
        if (_limit_flag)
        {
          // Stop motor
          setAcceleration(_default_accel * 10);
          stop();
          runToPosition();
          
          // Return motor _nsteps_return
          int step_dir = (relative>0) - (relative<0); // Current direction
          setAcceleration(_default_accel);
          move(_nsteps_return*-step_dir);
          runToPosition();
          setLimits();
          _limit_flag = 0; // Restart flag
        }

        if (_release_flag)
        {
          relative =0;
          _release_flag =0;
        }

      } while (distanceToGo() != 0);  
    }

  void BocciaStepper::releaseBall(long relative)
  {
    moveRun(relative);
    move(-70);
    runToPosition();

  }

  void BocciaStepper::setLimits()
  {
    // Set _limits for the first time
    if (_limits[0]==0 && _limits[1]==0)
    { 
      _limits[1] = currentPosition();
      Serial.println("No limits set, upper limit = " + String(_limits[1]));
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
      int curr_pos = currentPosition();

      // If higher limit was touched
      if (abs(_limits[1]-curr_pos) < abs(_limits[0]-curr_pos))
      {
        _limits[0] += curr_pos-_limits[1];
        _limits[1] = curr_pos; 
      }

      // Else, lower limit was touched
      else
      {
        _limits[1] += curr_pos-_limits[0];
        _limits[0] = curr_pos;
      }

      Serial.println("limits updated");
      Serial.println("- New upper limit = " + String(_limits[1]));
      Serial.println("- New lower limit = " + String(_limits[0]));
      }

  }

  void BocciaStepper::stopDetected()
  {
    _release_flag = 1;
    setAcceleration(10 * _default_accel);  // Change acceleration to stop quickly
    stop();
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

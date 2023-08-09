#include <BocciaStepper.h>
#include <AccelStepper.h>
#include <Functions.h>
  
BocciaStepper::BocciaStepper(
int pin_step,
int pin_dir,
int interrupt_pins[2],
int nsteps,
int nsteps_return,
int default_speed,
int default_accel,
bool use_limits,
int gear_ratio):
AccelStepper(AccelStepper::DRIVER, pin_step, pin_dir)
{
    _pin_step = pin_step;
    _pin_dir = pin_dir;
    _nsteps = nsteps;
    _nsteps_return = nsteps_return; 

    _default_speed = default_speed;
    _default_accel = default_accel;
    _gear_ratio = gear_ratio;
    _use_limits = use_limits;

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

    // If the requested value exceeds set limits, readjust
    requestedWithinLimits(relative);

    // Set movement and get there
    move(relative);
    do
    {
        run();

        // If limit detected, quickly stop and return motor
        if (_limit_flag) { clearSensorWhileMoving(active_interrupt_pin, relative); }
    } while (distanceToGo() != 0);  
}

  void BocciaStepper::limitDetected()
  {
    _limit_flag = 1;    
  }

void BocciaStepper::findRange()
{
    // Start with position at 0 and reset limits
    Serial.println("Reseting stepper position and limits");
    setCurrentPosition(0);
    limits[0] = 0;
    limits[1] = 0;

    moveDegrees(_nsteps);   // Find upper limit
    moveDegrees(-_nsteps);  // Find lower limit

      // Calculate mid point and move there
      // TODO: Implement this
      // float current_pos = currentPosition();
      float current_position = currentPosition();
      float midpoint = floor((limits[1]-limits[0])/2)+limits[0];
      Serial.println("Serial midpoint: " + String(midpoint));
      Serial.println("Serial current: " + String(current_position));
      
      moveToMiddle();
      Serial.println("End point: " + String(currentPosition()));
      
    }

  void BocciaStepper::moveToMiddle()
  {
    if (limits[1] && limits[0] ==0)
    {
      Serial.println("limits are not indicated");
      return;
    }  
    midpoint = floor((limits[1]-limits[0])/2)+limits[0];
    moveRun(long(abs(midpoint-currentPosition())));
  }

   
  void BocciaStepper::groundInputs()
  {
    digitalWrite(_pin_step, 0);
    digitalWrite(_pin_dir, 0);
}

void BocciaStepper::releaseBall(long degrees)
{
    moveDegrees(degrees);
    moveDegrees(-2*degrees);
}

void BocciaStepper::clearSensorWhileStopped(int pin)
{
    if (digitalReadDebounce(pin,5,1))
    {
        if (pin == _interrupt_pins[0]) { moveRun(_nsteps_return); }
        else if (pin == _interrupt_pins[1]) { moveRun(-_nsteps_return); }

        Serial.println("Pin " + String(pin) + " activated, clearing sensor");
        waitMillis(100);
    }
}
  
void BocciaStepper::moveDegrees(int degrees)
{
    int steps = int(floor(float(_gear_ratio)*float(degrees)*float(_nsteps)/360));
    moveRun(steps);
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

void BocciaStepper::requestedWithinLimits(long relative)
{
    // If we are using limits and they are set
    if (_use_limits && (limits[0]!=0 && limits[1]!=0))
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
}

void BocciaStepper::clearSensorWhileMoving(int pin, long relative)
{
    pin = active_interrupt_pin;

    // Check that the sensor has stabilized, and is active
    if (digitalReadDebounce(pin,5,1))
    {          
        // Stop motor
        Serial.println("Limit detected, stopping");
        setAcceleration(_default_accel * 10);
        stop();
        runToPosition();
        
        // Find current direction and return motor _nsteps_return
        int step_dir = signum(float(relative), 0.0);
        Serial.println("Returning " + String(-step_dir*_nsteps_return) + " steps");
        setAcceleration(_default_accel);            
        move(-step_dir*_nsteps_return);
        runToPosition();
        
        // Reset limits
        if (_use_limits) { setLimits(); }
    }
    _limit_flag = 0; // Restart flag
}
  
  
 

  
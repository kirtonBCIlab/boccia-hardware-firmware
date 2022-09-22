#ifndef BOCCIASTEPPER_H
#define BOCCIASTEPPER_H
#include <AccelStepper.h>
#include <Arduino.h>

class BocciaStepper:public AccelStepper
{
  private:
    int nsteps=200;           // Number of steps for a full rotation
    int nreturn = 1;          // Number of steps to return if limitDetected()
    int pin_interrupt = 2;    // Pin for interrupt
    int limits[2] = {0};      // Limits [low, high]
    bool homing_flag = 0;     // Homing flag (raised when homing is finished)
    bool limit_flag = 0;      // Limit flag (raised when an interrupt has activated)
    int default_speed = 200;  // Default speed [steps/sec]
    int default_accel = 10;   // Default acceleration [steps/(sec^2)]

    void setLimits();
    void setLowLimit();
    void setHighLimit();
  
  using AccelStepper::AccelStepper;
  
  public:
    void moveRun(long relative);

    void setReturnSteps(int steps);
    
    void setInterruptPin(int pin_sensor);
    int getInterruptPin();
    
    void setDefaultSpeed(int speed_val);
    int getDefaultSpeed();

    void setDefaultAccel(int accel_val);
    int getDefaultAccel();

    void setNoSteps(int steps);
    
    void limitDetected();
    void findRange();
};


#endif

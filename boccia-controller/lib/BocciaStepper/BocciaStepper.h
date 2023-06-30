#ifndef BOCCIASTEPPER_H
#define BOCCIASTEPPER_H
#include <AccelStepper.h>
#include <Arduino.h>

class BocciaStepper:public AccelStepper
{
  private:
    int _pin_step;                // Pin connected to step input of driver
    int _pin_dir;                 // Pin connected to dir input of driver
    int _interrupt_pins[2] = {0}; // List with pins connected to interrupts
    int _nsteps = 200;            // Number of steps for a full rotation
    int _nsteps_return = 1;       // Number of steps to return if limitDetected()
    int _default_speed;           // Default speed [steps/sec]
    int _default_accel;           // Default acceleration [steps/(sec^2)]

    int pin_interrupt = 2;    // Pin for interrupt
    int _limits[2] = {0};      // Limits [low, high]
    bool homing_flag = 0;     // Homing flag (raised when homing is finished)
    bool limit_flag = 0;      // Limit flag (raised when an interrupt has activated)
    int default_speed = 200;  // Default speed [steps/sec]
    int default_accel = 10;   // Default acceleration [steps/(sec^2)]

    void setLimits();
  
  // using AccelStepper::AccelStepper;
  
  public:
    BocciaStepper(int pin_step, int pin_dir, int interrupt_pins[2], int nsteps=200, int nsteps_return=5, int default_speed=200, int default_accel=10)
    : AccelStepper(AccelStepper::DRIVER, pin_step, pin_dir){};

    /// @brief Moves the stepper the desired amount of steps
    /// @param relative Number of steps that the stepper motor will move.
    /// positive numbers will rotate clockwise, negative numbers will rotate
    /// anticlockwise. The function checks if the motor range limits are set,
    /// if the limits are not set, they will get set based on the optical sensor.
    /// If the limits are already set, and the sensor is triggered, the limits
    /// are updated.
    void moveRun(long relative);
    
    /// @brief ISR activated when one of the optiocal sensors is triggered
    void limitDetected();

    /// @brief Moves the motor a full rotation clockwise, and then
    /// moves the motor a full rotation anticlocwise. 
    void findRange();
};


#endif

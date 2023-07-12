#ifndef BOCCIASTEPPER_H
#define BOCCIASTEPPER_H
#include <AccelStepper.h>
#include <Arduino.h>

class BocciaStepper:public AccelStepper
{
  private:
    int _pin_step;                // Pin connected to step input of driver
    int _pin_dir;                 // Pin connected to dir input of driver
    int _interrupt_pins[2] = {0}; // List with pins connected to interrupts [left, rigth]
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

    // BocciaStepper* bocciaStepper;
  
  // using AccelStepper::AccelStepper;
  
  public:
    /// @brief Creates a stepper motor object
    /// @param pin_step       Pin used for step input in stepper driver
    /// @param pin_dir        Pin used for direction input in stepper driver
    /// @param interrupt_pins List of pins used for interrupts [max 2] [left, right]   
    /// @param nsteps         Number of steps for a full rotation of stepper motor
    /// @param nsteps_return  Number of steps to return motor once it has it a limit
    /// @param default_speed  Default speed [steps/sec]
    /// @param default_accel  Default acceleration [steps/(sec^2)]
    BocciaStepper(int pin_step, int pin_dir, int interrupt_pins[2], int nsteps=200, int nsteps_return=5, int default_speed=200, int default_accel=10);

    /// @brief Initializes the pins associated with the motor to input or output
    /// accordingly.
    void initializePins();
    
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

    /// @brief Sets step and dir inputs to 0, to avoid any unwanted movements.
    void groundInputs();
};


#endif

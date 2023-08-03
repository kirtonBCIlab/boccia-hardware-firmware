#ifndef BOCCIASTEPPER_H
#define BOCCIASTEPPER_H
#include <AccelStepper.h>
#include <Arduino.h>

class BocciaStepper:public AccelStepper
{ 
    public:
        int active_interrupt_pin;    // Int to determine which pin activated the interrupt
        int limits[2] = {0};         // Step position of limits [low, high]

        /// @brief Creates a stepper motor object
        /// @param pin_step       Pin used for step input in stepper driver
        /// @param pin_dir        Pin used for direction input in stepper driver
        /// @param interrupt_pins List of pins used for interrupts [max 2] [left, right]   
        /// @param nsteps         Number of steps for a full rotation of stepper motor
        /// @param nsteps_return  Number of steps to return motor once it has it a limit
        /// @param default_speed  Default speed [steps/sec]
        /// @param default_accel  Default acceleration [steps/(sec^2)]
        /// @param gear_ratio     Gear ratio used in hardware mechanism [output:intput]
        BocciaStepper(
            int pin_step,
            int pin_dir,
            int interrupt_pins[2],
            int nsteps=200,
            int nsteps_return=5,
            int default_speed=200,
            int default_accel=10,
            bool use_limits=true,
            int gear_ratio=1
        );

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

        /// @brief ISR activated when one of the optical sensors is triggered
        void limitDetected();

        /// @brief Moves the motor a full rotation clockwise, anticlockwise, and
        ///        then places the motor at the midpoint. This call is used 
        ///        to set the range of motion based on the optical sensors 
        void findRange();

        /// @brief Sets step and dir inputs to 0, to avoid any unwanted movements.
        void groundInputs();

        /// @brief Moves a number of steps and returns in the opposite direction 
        ///        to hit sensor of release mechanism. 
        /// @param degrees Number of degrees that the motor will move to open the 
        ///                release mechanism.
        void releaseBall(long degrees);

        /// @brief Moves the motor off the sensor before calibration. 
        /// @param pin pin to check if the sensor is activated.
        void clearSensorWhileStopped(int pin);

        /// @brief Moves the stepper the desired amount of degrees.
        ///        Takes into to consideration nsteps and gear_ratio.
        /// @param degrees Number of degrees that the stepper motor will move.
        void moveDegrees(int degrees);

    private:
        int _pin_step;                // Pin connected to step input of driver
        int _pin_dir;                 // Pin connected to dir input of driver
        int _interrupt_pins[2] = {0}; // List with pins connected to interrupts [left, right]
        int _nsteps;                  // Number of steps for a full rotation
        int _nsteps_return;           // Number of steps to return if limitDetected()
        int _default_speed;           // Default speed [steps/sec]
        int _default_accel;           // Default acceleration [steps/(sec^2)]
        bool _use_limits;             // Bool to know whether to use limits [enable for motors with two optical sensors]
        int _gear_ratio;              // Gear ratio used in hardware mechanism [output:intput]

        bool _limit_flag = 0;         // Limit flag (raised when an interrupt has activated)

        /// @brief Set the values for the upper and lower limit for the
        ///        first time, or updates the values of the limits if one of the sensors
        ///        is touched while the motor is moving.
        void setLimits();

        /// @brief Check that the limit requested is within the current limits
        /// @param relative Relative position that the stepper is trying to get to [nsteps].
        void requestedWithinLimits(long relative);

        /// @brief Moves the motor off the sensor before calibration. 
        /// @param pin pin to check if the sensor is activated.
        /// @param relative Relative position that the stepper is trying to get to [nsteps].
        void clearSensorWhileMoving(int pin, long relative);
};
#endif

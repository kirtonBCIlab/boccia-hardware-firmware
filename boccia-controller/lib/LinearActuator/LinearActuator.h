#ifndef LinearActuator_H
#define LinearActuator_H
#include <AccelStepper.h>
#include <Arduino.h>

class LinearActuator
{
    private:
        int _pin1;                  // Pin 1 for motor driver, PWM driven
        int _pin2;                  // Pin 2 for motor driver, PWM driven
        int _pin_pot;               // Pin for the potentiometer
        int _pin_sensor;            // Pin for analog sensor [optional]
        float _stroke_length;       // Stroke length [in]
        int _pwm_speed;             // PWM speed [0 to 255]
        bool _homing_flag = 0;      // Homing flag (raised when homing is finished)
        int _limits[2] = {0};       // Limits [low, high]. Min = 0, Max = 2^10
        int _speed_threshold;       // Threshold to switch between full speed and speed_factor [%]
        int _speed_factor;          // Percentage to reduce speed to if requested movement < _speed_threshold [%]

        /// @brief Drives the actuator in the desired direction. Change
        /// the speed with _pwm_speed 
        /// @param direction Direction of the movement.
        /// 1: Extend actuator, 0: hold position, -1: retract actuator
        void driveActuator(int direction);

        /// @brief Converts resistance to a percentage value [0-100%]
        float resistanceToPercentage(float resistance);

        /// @brief Converts percentage [0-100%] to a resistance value
        float percentageToResistance(float percentage);

        /// @brief Drives the actuator to the desired limit
        /// @param direction Direction of the limit. 1: Extend, -1: Retract
        /// @return Analog reading of the potentiometer at the selected limit
        float moveToLimit(int direction);

    public:
        /// @brief Creates a linear actuator object
        /// @param pin1 PWM pin 1 to drive motor (Active = retract)
        /// @param pin2 PWM pin 2 to drive motor (Active = extend)
        /// @param pin_pot Analog pin to read potentiometer
        /// @param speed_threshold Threshold to switch between full speed and speed_factor [%]
        /// @param speed_factor Percentage to reduce speed to if requested movement < speed_threshold [%]
        /// @param pin_sensor Pin number for the force sensor [optional]
        LinearActuator(int pin1, int pin2, int pin_pot, int speed_threshold, int speed_factor, int pin_sensor=18);

        /// @brief Returns pin of the pressure sensor
        int getSensorPin();

        /// @brief Moves linear actuator to a percentage of the full range.
        /// Note: needs findRange() function to be run first.
        /// @param percentage Percentage of extension or retraction [0-100].
        void moveToPercentage(int percentage);

        void findRange();

        /// To use with interrupt service routine to detect limits when a force sensor is used.
        /// Returns the value of the potentiometer once the limit has been reached.
        float limitDetected();

};

#endif
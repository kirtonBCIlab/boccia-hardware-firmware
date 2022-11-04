#ifndef LINEARSTEPPER_H
#define LINEARSTEPPER_H
#include <AccelStepper.h>
#include <Arduino.h>

class LinearStepper
{
    private:
        int _pin1;                  // Pin 1 for motor driver, PWM driven
        int _pin2;                  // Pin 2 for motor driver, PWM driven
        int _pin_sensor;            // Pin for the potentiometer sensor 
        float _stroke_length;       // Stroke length [in]
        int _pwm_speed;             // PWM speed [0 to 255]
        bool _homing_flag = 0;      // Homing flag (raised when homing is finished)
        int _limits[2] = {0};       // Limits [low, high]. Min = 0, Max = 2^10

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
        LinearStepper(int pin1, int pin2, int pin_sensor, int pwm_speed);

        void setSensorPin(int pin);
        int getSensorPin();

        /// @brief Moves linear actuator to a percentage of the full range.
        /// Note: needs findRange() function to be run first.
        /// @param percentage Percentage of extension or retraction [0-100].
        void moveToPercentage(long percentage);

        void findRange();

};

#endif
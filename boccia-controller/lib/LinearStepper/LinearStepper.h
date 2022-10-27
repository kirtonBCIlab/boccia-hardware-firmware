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

        void driveActuator(int direction);
        float resistanceToPercentage(float resistance);
        float percentageToResistance(float percentage);
        float moveToLimit(int direction);

    public:
        LinearStepper(int pin1, int pin2, int pin_sensor, int pwm_speed);

        void setSensorPin(int pin);
        int getSensorPin();

        void moveToPercentage(float percentage);
        void findRange();

};

#endif
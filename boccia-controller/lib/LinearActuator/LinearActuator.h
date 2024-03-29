#ifndef LinearActuator_H
#define LinearActuator_H
#include <Arduino.h>

class LinearActuator
{
    public:
        /// @brief Creates a linear actuator object
        /// @param pin1 PWM pin 1 to drive motor (Active = retract)
        /// @param pin2 PWM pin 2 to drive motor (Active = extend)
        /// @param pin_pot Analog pin to read potentiometer
        /// @param speed_threshold Threshold to switch between full speed and speed_factor [%]
        /// @param speed_factor Percentage to reduce speed to if requested movement < speed_threshold [%]
        /// @param pin_sensor Pin number for the force sensor [optional]
        /// @param threshold Analog threshold value to consider pin_sensor active [0-1023]
        /// @param sensor_retract Time to retract motor if pin sensor is pressed [msec]
        LinearActuator(
            int pin1,
            int pin2,
            int pin_pot,
            int speed_threshold,
            int speed_factor,
            int pin_sensor=0,
            int threshold=500,
            int sensor_retract=750
            );

        /// @brief Initializes the pins associated with the motor to input or output
        /// accordingly.
        void initializePins();

        /// @brief Returns the pin number of the pressure sensor
        int getSensorPin();

        /// @brief Moves linear actuator to a percentage of the full range.
        ///        Note: needs ranges set first with findRange() or presetRange() function 
        ///        to be run first.
        /// @param percentage Percentage of extension or retraction [0-100].
        void moveToPercentage(int percentage);

        /// @brief Moves linear actuator to a percentage of the full range.
        ///        Note: needs ranges set first with findRange() or presetRange() function 
        ///        to be run first.
        /// @param percentage Percentage of extension or retraction [0-100].
        void moveToPercentageRange(int percentage);

        /// @brief Moves the linear actuator by a set percentage using the moveToPercentage
        ///        method. Note that this call requires the range of motion to be set.
        /// @param percentage percentage amount to move motor. >0=extension, <0=retraction.
        void moveByPercentage(int percentage);

        /// @brief Moves the linear actuator by a set percentage using the moveToPercentageRange
        ///        method. Note that this call requires the range of motion to be set.
        /// @param percentage percentage amount to move motor. >0=extension, <0=retraction.
        void moveByPercentageRange(int percentage);
        
        /// @brief Finds the range of movement of the motor. First, it retracts the
        ///        motor until it stops; then it extends the motor until it stops.
        ///        The range limits are set based on the potentiometer value during
        ///        the stop periods.
        ///        - Note that if there is a pressure sensor associated with the motor,
        ///          the motor first retracts to make sure the sensor is not pressed.
        void findRange();

        /// To use with interrupt service routine to detect limits when a force sensor is used.
        /// Returns the value of the potentiometer once the limit has been reached.
        float limitDetected();

        /// @brief Presets range values for potentiometer.
        ///        These values need to be measured before hand.
        /// @param lower_limit ADC value of the lower limit [0-1023]
        /// @param higher_limit ADC value of the higher limit [0-1023]
        void presetRange(int lower_limit, int higher_limit);

        /// @brief Drives actuator set time in the corresponding direction 
        void driveTime(int direction, int time);

    private:
        int _pin1;                  // Pin 1 for motor driver, PWM driven
        int _pin2;                  // Pin 2 for motor driver, PWM driven
        int _pin_pot;               // Pin for the potentiometer
        bool _pin_sensor_flag;      // Flag to know if sensor is not default value
        float _stroke_length;       // Stroke length [in]
        int _pwm_speed = 255;       // PWM speed [0 to 255]
        int _limits[2] = {0};       // Limits [low, high]. Min = 0, Max = 2^10
        int _speed_threshold;       // Threshold to switch between full speed and speed_factor [%]
        int _speed_factor;          // Percentage to reduce speed to if requested movement < _speed_threshold [%]
        int _pin_sensor;            // Pin for analog sensor [optional]
        int _analog_threshold;      // Analog threshold value to consider pin_sensor active [0-1023]
        int _sensor_retract;        // Time to retract motor if pin sensor is pressed [msec]
        int _dir_correction=1;      // Variable to correct movement of motor if pot is connected backwards
                                    // 1 = Regular direction
                                    // -1 = Invert direction for correct movement

        /// @brief Drives the actuator in the desired direction. Change
        /// the speed with _pwm_speed 
        /// @param direction Direction of the movement.
        ///                  1: Extend actuator, 0: hold position, -1: retract actuator
        void driveActuator(int direction);

        /// @brief Converts an ADC value to a percentage value [0-100%]
        float ADCToPercentage(float resistance);

        /// @brief Converts percentage [0-100%] to an ADC value [0-1023]
        float percentageToADC(float percentage);

        /// @brief Drives the actuator to the desired limit
        /// @param direction Direction of the limit. 1: Extend, -1: Retract
        /// @return Analog reading of the potentiometer at the selected limit
        float moveToLimit(int direction);

        /// @brief Finds whether the requested movement goes off limits [0-100]
        /// @param target_percentage Requested target percentage to move.
        int moveOverLimit(int target_percentage);

};

#endif
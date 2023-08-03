#include <LinearActuator.h>
#include <Functions.h>

LinearActuator::LinearActuator(int pin_1, int pin_2, int pin_pot, int speed_threshold, int speed_factor, int pin_sensor, int threshold, int sensor_retract)
{
    _pin1 = pin_1;
    _pin2 = pin_2;
    _pin_pot = pin_pot;
    _pin_sensor = pin_sensor;        
    _speed_threshold = speed_threshold;
    _speed_factor = speed_factor;
    _pin_sensor_flag = false;
    _analog_threshold = threshold;
    _sensor_retract = sensor_retract;

    // Enable sensor flag if _pin_sensor is not default
    if (_pin_sensor != 0) { _pin_sensor_flag = true; }   
}

void LinearActuator::initializePins()
{
    pinMode(_pin1, OUTPUT);
    pinMode(_pin2, OUTPUT);
    pinMode(_pin_pot, INPUT);
    if (_pin_sensor != 0) { pinMode(_pin_sensor, INPUT); }
}

void LinearActuator::driveActuator(int direction)
{
    switch(direction)
    {
        case 1:       //extension
            analogWrite(_pin2, _pwm_speed);
            analogWrite(_pin1, 0);
            break;
    
        case 0:       //stopping
            analogWrite(_pin2, 0);
            analogWrite(_pin1, 0);
            break;

        case -1:      //retraction
            analogWrite(_pin2, 0);
            analogWrite(_pin1, _pwm_speed);
            break;
    }
}

float LinearActuator::ADCToPercentage(float resistance)
{
    int range[2] = {0, 100};        // Range of motion for normalization [%]

    float num = range[1]*(resistance-_limits[0]) + range[0]*(_limits[1]-resistance);
    float den = _limits[1] - _limits[0];
    float percentage = num/den;

    return percentage;
}

float LinearActuator::percentageToADC(float percentage)
{
    int range[2] = {0, 100};        // Range of motion for normalization [%]

    float num = _limits[1]*(percentage-range[0]) + _limits[0]*(range[1]-percentage);
    float den = range[1] - range[0];
    float resistance = abs(num/den);

    return resistance;
}

float LinearActuator::moveToLimit(int direction)
{
    _pwm_speed = 255;           // Move to limit with full speed
    float prev_reading = 0.0;   // Previous reading ot the pot value
    float curr_reading = 0.0;   // Current reading of the po value
    bool sensor_pressed;        // Boolean to know whether the pressure se
    int wait_time = 400;        // Time to wait for the pot voltage to stabilize

    // Keep moving until the reading is stable for X msec or sensor active
    // If object was created with no sensor, the second part of the while check statement is discarded
    do{
        prev_reading = curr_reading;
        driveActuator(direction);
        waitMillis(wait_time);   

        curr_reading = analogRead(_pin_pot);
        Serial.println("Pot val: " + String(curr_reading));

        // If sensor exists, check when it's pressed, stop, and return waitMillis
        if (_pin_sensor_flag) 
        {
            sensor_pressed = analogReadDebounce(_pin_sensor, _analog_threshold, 5, 1);
            if (sensor_pressed) 
            { 
                driveActuator(0);
                driveActuator(-direction);
                waitMillis(_sensor_retract);
                Serial.println("Sensor pressed, stopping"); 
                break;
            }
        } 
    }while((prev_reading!=curr_reading) && !(_pin_sensor_flag && sensor_pressed));
    
    driveActuator(0);   // Stop actuator once you have reached the desired position
    waitMillis(100);
    curr_reading = analogRead(_pin_pot);

    return curr_reading;
}

void LinearActuator::findRange()
{   
    // Before calibration, if there is a press sensor retract motor
    // so that the sensor is not pressed
    if (_pin_sensor_flag)
    {
        driveActuator(-1);
        waitMillis(_sensor_retract*3);
    }

    Serial.println("Retracting...");
    _limits[0] = moveToLimit(-1);
    Serial.println("Retraction limit = " + String(_limits[0]));
    waitMillis(500);

    Serial.println("Extending...");
    _limits[1] = moveToLimit(1);
    Serial.println("Extension limit = " + String(_limits[1]));

    // If limits are backwards, drive the motor the opposite direction
    if (_limits[0]>_limits[1]) { _dir_correction = -1; }
}

void LinearActuator::presetRange(int lower_limit, int higher_limit)
{
    _limits[0] = lower_limit;
    _limits[1] = higher_limit;

    // If limits are backwards, drive the motor the opposite direction
    if (_limits[0]>_limits[1]) { _dir_correction = -1; }
    Serial.println("Manually set limits to " + String(_limits[0]) + " & " + String(_limits[1]));
}

void LinearActuator::moveToPercentage(int percentage)
{
    // First, avoid going over the limits
    moveOverLimit(percentage);

    // Find the direction to move
    float curr_reading = analogRead(_pin_pot);
    float resistance = percentageToADC(percentage); 
    int direction = signum(resistance, float(curr_reading))*_dir_correction;

    // If target is below threshold, reduce speed (i.e., fine movements)
    int move_percentage = abs(ADCToPercentage(curr_reading) - percentage);
    if (move_percentage <= _speed_threshold)
        {   _pwm_speed = int(floor(255 * _speed_factor / 100)); }
    else
        {   _pwm_speed = 255;   }
    
    // Move motor in correct direction until resistance is passed
    switch (direction)
    {
        case 1:
            while (curr_reading < resistance)
            {
                driveActuator(direction);
                curr_reading = analogRead(_pin_pot);
            }
            break;
        case -1:
            while (curr_reading > resistance)
            {
                driveActuator(direction);
                curr_reading = analogRead(_pin_pot);
            }
            break;
        case 0:
            break;
    }

    driveActuator(0); // Stop motor
    waitMillis(100);
}

void LinearActuator::moveToPercentageRange(int percentage)
{
    // First, avoid going over the limits
    moveOverLimit(percentage);

    // Take initial reading to determine movement direction
    float curr_reading = analogRead(_pin_pot);
    Serial.println("Current: " + String(curr_reading));
    float resistance = percentageToADC(percentage); 
    Serial.println("Target: " + String(resistance));
    int direction = signum(resistance, float(curr_reading))*_dir_correction;
    Serial.println("Direction: " + String(direction));

    // If target is below threshold, reduce speed (i.e., fine movements)
    int move_percentage = abs(ADCToPercentage(curr_reading) - percentage);
    if (move_percentage <= _speed_threshold)
        {   _pwm_speed = int(floor(255 * _speed_factor / 100)); }
    else
        {   _pwm_speed = 255;   }


    bool position_reached = false;  // Flag to know whether the requested position has been reached
                                    // 1 = Position reached, stop motor
                                    // 0 = Position not reached, move motor
    int num_readings = 4;           // Number of readings to potentiometer to be averaged
    int buffer = 2;                 // Buffer to the ADC values, this gives a range for error in 
                                    // the potentiometer measurements
    Serial.println("About to move motor");
    // Move motor until position is reached
    while (!position_reached)
    {
        curr_reading = 0;

        for (int i=0; i<num_readings; i++) { curr_reading = curr_reading + analogRead(_pin_pot); }
        curr_reading = curr_reading / num_readings;

        if (curr_reading < (resistance-buffer)) { driveActuator(1*_dir_correction);  }
        else if (curr_reading > (resistance+buffer)) { driveActuator(-1*_dir_correction);  }
        else 
        {
            Serial.println("stop");
            driveActuator(0);
            position_reached = true;
        }

    }

    Serial.println("Moved to " + String(analogRead(_pin_pot)));
}

int LinearActuator::getSensorPin()
{
    return _pin_sensor;
}

float LinearActuator::limitDetected()
{
    driveActuator(0);                           // Stop motor
    float curr_reading = analogRead(_pin_pot);  // Read potentiometer
    Serial.println("Sensor pressed");

    return curr_reading;
}

void LinearActuator::driveTime(int direction, int time)
{
    driveActuator(direction);
    waitMillis(time);
    driveActuator(0);
}

int LinearActuator::moveOverLimit(int target_percentage)
{
    if (target_percentage>100)
    {   
        Serial.println("Movement requested over 100%, will limit to 100%");
        target_percentage = 100;   
    }
    else if (target_percentage<0)
    {   
        Serial.println("Movement requested under 0%, will limit to 0%");
        target_percentage = 0;     
    }

    return target_percentage;
}

void LinearActuator::moveByPercentage(int percentage)
{
    if (_limits[0]==0 && _limits[1]==0)
    {
        Serial.println("No limits set, please set limits first");
        return;
    }

    // Calculate target percentage
    float curr_reading = analogRead(_pin_pot);
    int target_percentage = ADCToPercentage(curr_reading) + percentage;

    moveToPercentage(target_percentage);
}

void LinearActuator::moveByPercentageRange(int percentage)
{
    if (_limits[0]==0 && _limits[1]==0)
    {
        Serial.println("No limits set, please set limits first");
        return;
    }
    
    // Calculate target percentage
    float curr_reading = analogRead(_pin_pot);
    int target_percentage = int(floor(ADCToPercentage(curr_reading))) + percentage;

    moveToPercentageRange(target_percentage);
}

    
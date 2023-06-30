#include <LinearActuator.h>
#include <Functions.h>

LinearActuator::LinearActuator(int pin_1, int pin_2, int pin_pot, int speed_threshold, int speed_factor, int pin_sensor)
{
    _pin1 = pin_1;
    _pin2 = pin_2;
    _pin_pot = pin_pot;
    _pin_sensor = pin_sensor;        
    _speed_threshold = speed_threshold;
    _speed_factor = speed_factor;
    _pin_sensor_flag = false;

    // Enable sensor flag if _pin_sensor is not default
    if (_pin_sensor != 0)
    {
        _pin_sensor_flag = true;
        Serial.println("Pin sensor active");
    }   
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

float LinearActuator::resistanceToPercentage(float resistance)
{
    int range[2] = {0, 100};        // Range of motion for normalization [%]

    float num = range[1]*(resistance-_limits[0]) + range[0]*(_limits[1]-resistance);
    float den = _limits[1] - _limits[0];
    float percentage = num/den;

    return percentage;
}

float LinearActuator::percentageToResistance(float percentage)
{
    int range[2] = {0, 100};        // Range of motion for normalization [%]

    float num = _limits[1]*(percentage-range[0]) + _limits[0]*(range[1]-percentage);
    float den = range[1] - range[0];
    float resistance = num/den;

    return resistance;
}

float LinearActuator::moveToLimit(int direction)
{
    _pwm_speed = 255;   // Move to limit with full speed
    int prev_reading = 0;
    int curr_reading = 0;
    bool sensor_pressed;

    // Keep moving until the reading is stable for X msec or sensor active
    // If object was created with no sensor, the second part of the while check statement is discarded
    do{
        prev_reading = curr_reading;
        driveActuator(direction);
        waitMillis(250);   
        curr_reading = analogRead(_pin_pot);

        // If sensor exists, check when it's pressed
        if (_pin_sensor_flag) { sensor_pressed = digitalReadDebounce(_pin_sensor, 20, 1); }
        
    }while((prev_reading!=curr_reading) && !(_pin_sensor_flag && sensor_pressed));

    driveActuator(0);   // Stop actuator once you have reached the limit

    return curr_reading;
}

void LinearActuator::findRange()
{   
    Serial.println("Retracting...");
    _limits[0] = moveToLimit(-1);
    Serial.println("Retraction limit = " + String(_limits[0]));
    waitMillis(2000);

    Serial.println("Extending...");
    _limits[1] = moveToLimit(1);
    Serial.println("Extension limit = " + String(_limits[1]));
    waitMillis(2000);
}

void LinearActuator::moveToPercentage(int percentage)
{
    // First, avoid going over the limits
    if (percentage>100)
    {   
        Serial.println("Movement requested over 100%, will limit to 100%");
        percentage = 100;   
    }
    else if (percentage<0)
    {   
        Serial.println("Movement requested under 0%, will limit to 0%");
        percentage = 0;     
    }

    int curr_reading = analogRead(_pin_pot);
    Serial.println("Current: " + String(curr_reading));
    float resistance = percentageToResistance(percentage); 
    Serial.println("Target: " + String(resistance));
    int direction = signum(resistance, float(curr_reading));

    // If target is below threshold, reduce speed (i.e., fine movements)
    int move_percentage = abs(resistanceToPercentage(curr_reading) - percentage);
    if (move_percentage <= _speed_threshold)
        {   _pwm_speed = int(floor(255 * _speed_factor / 100)); }
    else
        {   _pwm_speed = 255;   }


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


    
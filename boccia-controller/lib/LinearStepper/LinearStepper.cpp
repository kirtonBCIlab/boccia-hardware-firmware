#include <LinearStepper.h>
#include <Functions.h>

    LinearStepper::LinearStepper(int pin_1, int pin_2, int pin_sensor, int pwm_speed)
    {
        _pin1 = pin_1;
        _pin2 = pin_2;
        _pin_sensor = pin_sensor;
        _pwm_speed = pwm_speed;
    }

    void LinearStepper::driveActuator(int direction)
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

    float LinearStepper::resistanceToPercentage(float resistance)
    {
        int range[2] = {0, 100};        // Range of motion for normalization [%]

        float num = range[1]*(resistance-_limits[0]) + range[0]*(_limits[1]-resistance);
        float den = _limits[1] - _limits[0];
        float percentage = num/den;

        return percentage;
    }

    float LinearStepper::percentageToResistance(float percentage)
    {
        int range[2] = {0, 100};        // Range of motion for normalization [%]

        float num = _limits[1]*(percentage-range[0]) + _limits[0]*(range[1]-percentage);
        float den = range[1] - range[0];
        float resistance = num/den;

        return resistance;
    }

    float LinearStepper::moveToLimit(int direction)
    {
        int prev_reading = 0;
        int curr_reading = 0;

        do{
            prev_reading = curr_reading;
            driveActuator(direction);
            waitMillis(200);    // Keep moving until the reading is stable for 200 msec
            curr_reading = analogRead(_pin_sensor);
        }while(prev_reading != curr_reading);

        return curr_reading;
    }

    void LinearStepper::findRange()
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

    void LinearStepper::moveToPercentage(long percentage)
    {
        int curr_reading = analogRead(_pin_sensor);
        Serial.println("Current: " + String(curr_reading));
        float resistance = percentageToResistance(percentage); 
        Serial.println("Target: " + String(resistance));
        int direction = signum(resistance, float(curr_reading));

        switch (direction)
        {
            case 1:
                while (curr_reading < resistance)
                {
                    driveActuator(direction);
                    curr_reading = analogRead(_pin_sensor);
                }
                break;
            case -1:
                while (curr_reading > resistance)
                {
                    driveActuator(direction);
                    curr_reading = analogRead(_pin_sensor);
                }
                break;
            case 0:
                break;
        }

        driveActuator(0); // Stop motor
        waitMillis(100);

        Serial.println("Moved to " + String(analogRead(_pin_sensor)));

    }


    
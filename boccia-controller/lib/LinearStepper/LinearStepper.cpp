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
        float a = 0.0;
        return a;
    }

    float LinearStepper::moveToLimit(int direction)
    {
        int prev_reading=0;
        int curr_reading=0;

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
        Serial.println("Extending...");
        float a = moveToLimit(1);
        Serial.println(String(a));
        waitMillis(500);
        Serial.println("Retracting...");
        float b = moveToLimit(-1);
        Serial.println(String(b));
    }

    void LinearStepper::moveToPercentage()
    {

    }


    
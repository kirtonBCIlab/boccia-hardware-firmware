// Include libraries
#include <Arduino.h>

// -------------------- Settings -------------------- //
// - Values for elevation motor and potentiometer
int pin1 = 9;       // Driver pin to retract
int pin2 = 10;      // Driver pin to extend
int pin_pot = 3;    // Analog pin connected to potentiometer
int limits[2] = {}; // Limits of the potentiometer [0-1024]
int automatic_limits[2] = {8, 331}; // Limits of the potentiometer 
                                    // Do NOT run manual calibration if using automatic limits

// - PID controller variables
unsigned long lastTime = 0; // Time of last computation for PID controller
int Input = 0.0;            // Sensor input value (i.e., position of potentiometer)
double Output = 0.0; // Control output (e.g., motor speed)
double Setpoint = 100.0; // Desired setpoint (adjust as needed)
double errSum = 0.0; // Integral term
double lastErr = 0.0; // Previous error

// - PID tuning parameters
double kp = 0.1;    // Proportional gain
double ki = 0.01;   // Integral gain
double kd = 0.001;  // Derivative gain

// --------------- Prototype functions --------------- //
/// @brief Calibrates the motor by moving it to the upper and lower limits.
///        The limits are stored in the limits array.
void manualCalibration();

/// @brief Sets the limits of the motor automatically
void automaticCalibration();

/// @brief Computes the PID output based on the input and setpoint
/// @param controller_type Type of controller to be used. 
///                        Options: "P", "PI", "PD", "PID"
void computePID(String controller_type, double setPoint);

/// @brief Drives motor based on direction and PWM speed
/// @param direction Direction of motor rotation 
///                 Options: 1 (extend), -1 (retract), 0 (stop)
/// @param pwm_speed PWM speed to drive motor
///                 0 = stop, 255 = full speed
void driveMotor(int direction, int pwm_speed);

/// @brief Moves motor to limit in a given direction
/// @param direction Direction of motor rotation
///                 Options: 1 (extend), -1 (retract) , 0 (stop)
/// @param pwm_speed PWM speed to drive motor
///                 0 = stop, 255 = full speed
int moveToLimit(int direction, int pwm_speed);

/// @brief Moves motor to the middle of the limits
///         This method uses the old move mechanism
void moveToMiddle();

/// @brief Waits for a given amount of time without blocking the program
/// @param wait_msec Time to wait [msec]
void waitMillis(unsigned long wait_msec);


// ---------------------- Setup ---------------------- //
void setup() 
{
    // Serial communication
    Serial.begin(9600);
    Serial.println("Starting program");

    // Calibrate motor - Choose one
    // manualCalibration();
    automaticCalibration();

    // Move to middle to start 
    moveToMiddle();

    
}

// ---------------------- Loop ---------------------- //
void loop() 
{
    // Read sensor input (e.g., analogRead, digitalRead)
    // Input = /* Read your sensor value here */;

    // Compute PID output
    // computePID();

    
    // Apply the control output (e.g., analogWrite, digitalWrite)
    // Adjust your motor speed or other actuator accordingly
}

void manualCalibration() 
{
    // Settings
    int pwm_speed = 255;        // Move to limit with full speed

    // Retract motor to find lower limit
    Serial.println("Manual calibration started");
    Serial.println("Retracting...");
    limits[0] = moveToLimit(-1, pwm_speed);

    // Extend motor to find upper limit
    Serial.println("Extending...");
    limits[1] = moveToLimit(1, pwm_speed);  
    Serial.println("Limits: " + String(limits[0]) + " - " + String(limits[1]));

    Serial.println("Manual calibration finished\n");
}

void automaticCalibration() 
{
    Serial.println("Automatic calibration started");
    limits[0] = automatic_limits[0];
    limits[1] = automatic_limits[1];
    Serial.println("Limits: " + String(limits[0]) + " - " + String(limits[1]));
    Serial.println("Automatic calibration finished\n");
}

void computePID(String controller_type, double setPoint) {
    // Calculate time since last computation
    unsigned long now = millis();
    double timeChange = (double)(now - lastTime);

    // Compute error terms
    double error = Setpoint - Input;
    errSum += (error * timeChange);
    double dErr = (error - lastErr) / timeChange;

    // Compute PID output
    Output = kp * error + ki * errSum + kd * dErr;

    // Remember variables for next iteration
    lastErr = error;
    lastTime = now;
}

void driveMotor(int direction, int pwm_speed) 
{
    switch(direction)
    {
        case 1:       //extension
            analogWrite(pin2, pwm_speed);
            analogWrite(pin1, 0);
            break;
    
        case 0:       //stopping
            analogWrite(pin2, 0);
            analogWrite(pin1, 0);
            break;

        case -1:      //retraction
            analogWrite(pin2, 0);
            analogWrite(pin1, pwm_speed);
            break;
    }
}

int moveToLimit(int direction, int pwm_speed) 
{
    // Set initial values
    int prev_reading = 0;   // Previous reading ot the pot value [0-1024]
    int curr_reading = 0;   // Current reading of the pot value [0-1024]
    int wait_time = 400;    // Time to wait for the pot voltage to stabilize [msec]

    // Keep moving until the reading is stable for wait_time [msec]
    do{
        prev_reading = curr_reading;
        driveMotor(direction, pwm_speed);
        waitMillis(wait_time);   

        curr_reading = analogRead(pin_pot);
    }while(prev_reading!=curr_reading);

    return curr_reading;
}

void moveToMiddle()
{
    // Calculate middle point
    int middle = (limits[1] - limits[0])/2;
    int curr_reading = analogRead(pin_pot);
    int direction = (curr_reading < middle) ? 1 : -1;
    int pwm_speed = 255;

    // Move to middle
    while ((direction == 1 && curr_reading < middle) || (direction == -1 && curr_reading > middle))
    {
        driveMotor(direction, pwm_speed);
        curr_reading = analogRead(pin_pot);
    }
    
    // Stop motor
    driveMotor(0, 0);
}

void waitMillis(unsigned long wait_msec)
{
    unsigned long deltaT, t0, t1 = 0;
    t0 = millis();
    
    do
    {
      t1 = millis();
      deltaT = t1-t0;
    }while(deltaT < wait_msec);
}
// Linear actuator test
// - This sketch can be used to test either the elevation or rotation actuator
//   for proper calibration.
// - Note that you must change the values in the setup section to control the
//   appropriate motor
// -- Elevation: pin1=9, pin2=10, pin_pot=3
// -- Rotation: pin1=7, pin2=8, pin_pot=4 

// Private variables
  // Settings for motor to be tested
int repeat = 1;       // Number of times to repeat calibration [n] 
int pin1 = 7;         // Input 1 of driver
int pin2 = 8;        // Input 2 of driver 
int pin_pot = 4;      // Analog pin connected to the potentiometer
int pwm_speed = 255;  // Speed of PWM to drive motor [0 - 255]

void setup() 
{
  Serial.begin(9600);
}

// Drives the actuator in the set direction
void driveActuator(int direction)
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

// Moves the actuator to the set limit
float moveToLimit(int direction)
{
    pwm_speed = 255;   // Move to limit with full speed
    int prev_reading = 0;
    int curr_reading = 0;
    bool sensor_pressed;

    // Keep moving until the reading is stable for X msec or sensor active
    // If object was created with no sensor, the second part of the while check statement is discarded
    do{
        prev_reading = curr_reading;
        driveActuator(direction);
        waitMillis(250);   
        curr_reading = analogRead(pin_pot);
        Serial.println(" - current value: " + String(curr_reading));
    }while(prev_reading!=curr_reading);

    driveActuator(0);   // Stop actuator once you have reached the limit

    return curr_reading;
}

// Wait the set time in [msec]
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

void loop() {
  while(repeat > 0)
  {
    Serial.println("Retracting sensor");
    moveToLimit(-1);
    waitMillis(2000);
  
    Serial.println("Extending sensor");
    moveToLimit(1);
    waitMillis(2000);

    repeat--;
  }
  
}

/*
  Actuator Feedback Test
  - This script is used to test the calibration of the actuator feedback
  - First, the actuator is calibrated with the built-in potentiometer
  - Second, the Arduino waits for a Serial command to extend or retract the actuator
  -- Commands should be as follows
  --- R = retract
  --- E = extend
  --- 1-3 = Amount to retract or extend [movement in 100s of msec]
  -- For example, if you want 200 msec extension, type "E2" in the serial monitor
*/
 
#include <elapsedMillis.h>
elapsedMillis timeElapsed;

// Setup variables
int r_pwm = 2;   
int l_pwm = 3;
int sensor_pin = A0;

int sensor_val;
int pwm_speed = 255;      // PWM duty cycle [0-255]
float stroke_len = 2.0;   // Customize to your specific stroke length [in]
float extension_len;

int max_analog_reading;
int min_analog_reading;

const int command_size = 2; // Number of characters in each command
char command[command_size]; // Array to store the characters received

void setup() {
  pinMode(r_pwm, OUTPUT);
  pinMode(l_pwm, OUTPUT);
  pinMode(sensor_pin, INPUT);
  
  Serial.begin(9600);
  Serial.println("Start calibration");
  Serial.println("Extending...");
  max_analog_reading = moveToLimit(1);
  Serial.println("Retracting...");
  min_analog_reading = moveToLimit(-1);
  Serial.println("Calibration finished");
  Serial.println("Type desired command");
}

void loop(){
  int idirection;// direction to move motor. 1=Extend, 0=hold, -1=Retract

  // Set movement within limits
  if (Serial.available()>0)
  {
    waitTime(100);  // Wait to get full serial readings
    //decodeSerial(); // Get command as a char array
    decodeCommand(command_size);
    idirection = decodedirection(command[0]);
    sensor_val = analogRead(sensor_pin);

    Serial.println("Start to move motor");
    for (int i=0; i<int(command[1] - '0'); i++)
    {
      if (sensor_val==min_analog_reading || sensor_val==max_analog_reading)
      {
        break;
      }
      else
      {
        driveActuator(idirection, pwm_speed); // Activate motor
        waitTime(100);              // Wait [msec]
      }
    }
    Serial.println("Stop motor");
    driveActuator(0, pwm_speed);  // Stop motor until next command
  }
}

void waitTime(unsigned long wait_msec)
{
    unsigned long deltaT, t0, t1 = 0;
    t0 = millis();
    
    do
    {
      t1 = millis();
      deltaT = t1-t0;
    }while(deltaT < wait_msec);

    Serial.println("Wait time: " + String(deltaT));
}

void decodeCommand(int num_bytes)
{
  char c;

  for (int index=0; index<=num_bytes; index++)
  {
    if (index == num_bytes)
    {
      c = Serial.read();
      Serial.println("Command finished");
      Serial.println("Complete command: " + String(command));
    }
    else
    {   
      c = Serial.read();
      command[index] = c;
      Serial.println("Index: " + String(index));
      Serial.println("Serial read: " + String(c));
    }
  }
}


void decodeSerial()
{
  int index = 0;
  char c;
  while (Serial.available() > 0)
  {
    c = (char) Serial.read();
    if(c == '\n')
    {
      Serial.println("Command read: ");
      for (int i=0;i==1;i++)
      {Serial.print(command[i]);}
      Serial.print('\n');
    }
    else
    {
      Serial.println("Character: " + c);
      command[index] = c;
      index++;
    }
  }
}

int decodedirection(char char_dir)
{
  if (char_dir == 'E')
  {
    return 1;
  }
  else if (char_dir == 'R')
  {
    return -1;
  }
}

int moveToLimit(int direction){
  int prevReading=0;
  int currReading=0;

  do{
    prevReading = currReading;
    driveActuator(direction, pwm_speed);
    timeElapsed = 0;
    while(timeElapsed < 200){delay(1);}           //keep moving until analog reading remains the same for 200ms
    currReading = analogRead(sensor_pin);
  }while(prevReading != currReading);
  return currReading;
}

float mapfloat(float x, float inputMin, float inputMax, float outputMin, float outputMax)
{
 return (x-inputMin)*(outputMax - outputMin)/(inputMax - inputMin)+outputMin;
}

void displayOutput()
{
  sensor_val = analogRead(sensor_pin);
  extension_len = mapfloat(sensor_val, float(min_analog_reading), float(max_analog_reading), 0.0, stroke_len);
  Serial.print("Analog Reading: ");
  Serial.print(sensor_val);
  Serial.print("\tActuator extension length: ");
  Serial.print(extension_len);
  Serial.println(" inches");  
}

void driveActuator(int direction, int pwm_speed)
{
  switch(direction)
  {
    case 1:       //extension
      analogWrite(r_pwm, pwm_speed);
      analogWrite(l_pwm, 0);
      break;
   
    case 0:       //stopping
      analogWrite(r_pwm, 0);
      analogWrite(l_pwm, 0);
      break;

    case -1:      //retraction
      analogWrite(r_pwm, 0);
      analogWrite(l_pwm, pwm_speed);
      break;
  }
}

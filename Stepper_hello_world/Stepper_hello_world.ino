/*
 * Stepper Hello World
 * - This scipt takes uses two pins to control a stepper motor
 */

const int num_steps = 200;
int num_turns = 2;      // Number of turns in each direction
int turn_duration = 5;  // Turn duration in seconds
int turn_direction = 0; // Boolean for turn direction
float wait_step;        // Time to wait for each step [usec]

const int pin_step = 2;
const int pin_dir = 3;

void setup() 
{
  pinMode(pin_step, OUTPUT);
  pinMode(pin_dir, OUTPUT); 

  wait_step = 1e6 * turn_duration / (2*num_steps);
}

void loop() 
{
  digitalWrite(pin_dir, turn_direction);


  for(int i=0; i<=num_turns*num_steps; i++)
  {
    digitalWrite(pin_step, HIGH);
    waitMicros((unsigned long)wait_step);
    digitalWrite(pin_step, LOW);
    waitMicros((unsigned long)wait_step);
  }

  waitMillis(500);

  turn_direction = !turn_direction;
}

void waitMicros(unsigned long wait_usec)
{
    unsigned long deltaT, t0, t1 = 0;
    t0 = micros();
    
    do
    {
      t1 = micros();
      deltaT = t1-t0;
    }while(deltaT < wait_usec);
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

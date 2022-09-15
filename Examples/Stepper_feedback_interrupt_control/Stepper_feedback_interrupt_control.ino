/*
 * Stepper Feedback Interrupt Control
 */


// Sensor variables
const int pin_sense = 2;  // Digital pin for digital interrupt

// Stepper motor variables
#include <AccelStepper.h>

const int pin_dir = 52;  // Direction pin
const int pin_step = 53; // Step pin
int lim0;               // Lower limit of the stepper motor [steps]
int lim1;               // Higher limit of the stepper motor [steps]
int range;              // Range of movement of the stepper [steps]
int dir = 1;
volatile bool switch_triggered;
int loop_no = 0;

// - Create stepper object
//AccelStepper nema8(AccelStepper::DRIVER, pin_step, pin_dir);
AccelStepper nema8 = AccelStepper(1, pin_step, pin_dir);

void setup()  {
  switch_triggered = false;
  
  nema8.setMaxSpeed(1000);    // Maximum speed [steps/sec]
  nema8.setAcceleration(10);  // Acceleration [steps/sec^2]
  nema8.setSpeed(400);        // Speed [steps/sec]

  pinMode(pin_sense, INPUT);
//  attachInterrupt(digitalPinToInterrupt(pin_sense), opticalSwitchTriggered, RISING);
  
  Serial.begin(9600);
  Serial.println("Start moving");
  
}

void loop() {
 moveMotor();  
}

void moveMotor()  {
  switch_triggered = false;
  //nema8.setSpeed(400);
  nema8.move(200 * dir);

  
  Serial.println("Target position: " + String(nema8.targetPosition()));

  while (nema8.distanceToGo() != 0) {
    nema8.run();
    if (digitalRead(pin_sense) == 1){
      Serial.println("Sensor triggered");
      Serial.println(" - Position: " + String(nema8.currentPosition()));
      Serial.println(" - Steps to go: " + String(nema8.distanceToGo()));
      Serial.println(" - Target position: " + String(nema8.targetPosition()));
      nema8.stop();
      waitMillis(1000);
//      nema8.moveTo(nema8.currentPosition());
      nema8.setCurrentPosition(0);
//      nema8.setSpeed(0);
      Serial.println(" - New target position: " + String(nema8.targetPosition()));
      Serial.println(" - New distance to go: " + String(nema8.distanceToGo()));
      switch_triggered = true;
      //break;
    }    
  }
  
  //Serial.println("Movement done");
  waitMillis(2000);
  dir = dir * -1;
  Serial.println("Direction: " + String(dir));
}

//void opticalSwitchTriggered(){
//  Serial.println("Switch triggered");
//  switch_triggered = true;
//  Serial.println("Motor stopped");
////  dir = dir * -1;
////  Serial.println("Remaining steps: " + String(nema8.distanceToGo()));
//}


//void setup() {
//  
//  
//  Serial.begin(9600);
//
//  // Set stepper parameters
//  nema8.setMaxSpeed(1000);    // Maximum speed [steps/sec]
//  nema8.setAcceleration(10);  // Acceleration [steps/sec^2]
//  nema8.setSpeed(400);        // Speed [steps/sec]
//  range = 0;                  // Range of motion of calibrated motor [steps]
//
//  Serial.println("Select command:\n - h: homing\n - m: move");
//}
//
//void loop() {
//  if(Serial.available() > 0)
//  {
//    char answer = Serial.read();
//    
//    // Flush buffer
//    for(int i=0; i<Serial.available(); i++)
//    { Serial.read();}
//    
//    
//    if(answer=='h')
//    {
//      homingProcess();
//      Serial.println("Select 'm' to move");
//    }
//    else if(answer=='m')
//    {
//      Serial.println("How many steps to move?");
//
//      while(Serial.available()<=1)
//      {
//        waitMillis(100);
//      }
//      int steps = Serial.parseInt();
//      moveProcess(lim0, lim1, steps); 
//    }
//    else
//    {}
//  }
//  
//}
//
//// Move stepper only within limits
//void moveProcess(int lim0, int lim1, int move_steps)
//{
//  int current_position = nema8.currentPosition();
//  Serial.println("Current pos: " + String(current_position));
//  Serial.println("How many steps requested? " + String(move_steps));
//  int new_position = current_position + move_steps;
//  Serial.println("Calculated new pos: " + String(new_position));
//
//  if((new_position < lim0) && (new_position > lim1))
//  {
//    moveStepper(move_steps);
//  }
//  else if (new_position > lim0)
//  {
//    move_steps = lim0 - current_position;
//    moveStepper(move_steps);
//  }
//  else if (new_position < lim1)
//  {
//    move_steps = lim1 - current_position;
//    moveStepper(move_steps);
//  }
//  else
//  {
//    Serial.println("No need to move, already at limit!");
//  }
//  Serial.println("Actual position after movement " + String(nema8.currentPosition()));
//}
//
//void homingProcess()
//{
//  Serial.println("Homing started");
//
//  nema8.move(200);  // Do a full turn
//  while (nema8.distanceToGo() != 0)
//  {
//    nema8.run();
//  }
//}
//
//// Calibrate stepper motor and return position 
//int calibrateStepper(int stepper_direction)
//{
//  float analog_value;
//  do
//  {
//    nema8.move(25*stepper_direction);
//    nema8.run();
//    analog_value = analogReader(pin_sense);
//  }while(analog_value < analog_threshold);
//
//  nema8.stop();
//  //nema8.move(0);  // Make sure the motor doesn't move after calibrating it
//  //nema8.run;
//  int stepper_lim = nema8.currentPosition();
//  return stepper_lim;
//}
//
//void moveStepper(int steps)
//{
//  nema8.move(steps);
//  while(nema8.distanceToGo() != 0)
//  {    
//    nema8.run();
//  }
//}
//
//// Read analog pin and convert to V
//float analogReader(int analog_pin)
//{
//  float analog_digital = analogRead(analog_pin);
//  float analog_volts = analog_digital * 5 / 1024;
//  return analog_volts;
//}

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

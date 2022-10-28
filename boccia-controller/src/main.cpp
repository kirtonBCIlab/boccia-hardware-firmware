// Include libraries
#include <Arduino.h>
#include <BocciaStepper.h>
#include <LinearStepper.h>
#include <Functions.h>
#include <AccelStepper.h>

int n_steps = 200;
int dir = 1;

// Define incline actuator
int pin1 = 4;
int pin2 = 5;
int pin_sensor = 0;
int pwm_speed = 255;
LinearStepper inclineActuator(pin1, pin2, pin_sensor, pwm_speed);

// Define Nema8
int pin_step = 52;
int pin_dir = 53;
BocciaStepper nema8(AccelStepper::DRIVER, pin_step, pin_dir);

// Prototype functions
void nema8Limit();
void waitMillis(unsigned long wait_msec);
void decodeCommand();

void setup() {
  // Serial communication debugging
  Serial.begin(9600);
  Serial.println("Begin setup");

  // Nema 8 settings
  nema8.setNoSteps(200);
  nema8.setReturnSteps(10);
  nema8.setDefaultSpeed(400);   // Default speed [steps/sec]
  nema8.setDefaultAccel(10);
  nema8.setMaxSpeed(1000);    // Maximum speed [steps/sec]
  nema8.setInterruptPin(2);
  nema8.setNoSteps(200);      // Number of steps for complete rotation [steps]

  // Set NEMA inputs to ground
  digitalWrite(pin_dir, 0);
  digitalWrite(pin_step, 0);

  // Interrupts
  attachInterrupt(digitalPinToInterrupt(nema8.getInterruptPin()), nema8Limit, RISING);

  Serial.println("Calibrating - NEMA8");
  nema8.findRange();
  Serial.println("NEMA8 - Calibration ended");

  Serial.println("Calibrating -  linear actuator");
  inclineActuator.findRange();
  Serial.println("Linear actuator - Calibration ended");

  Serial.println("Select motor and movement...");
}

void loop() 
{
  if (Serial.available())
  {
    decodeCommand();
  }
  // if (Serial.available())
  // {
  //   n_steps = Serial.parseInt();
    
  //   // Empty serial port
  //   for (int n=0; n<Serial.available(); n++)
  //   {
  //     Serial.read();
  //   }
    
  //   inclineActuator.moveToPercentage(n_steps);
  //   // Serial.println("Current position: " + String(nema8.currentPosition()));
  //   // Serial.println("Moving " + String(n_steps) + " steps");
  //   // nema8.moveRun(n_steps);

  // }
  waitMillis(250);
}

void nema8Limit()
{
  nema8.limitDetected();
}

void decodeCommand()
{
  long command = Serial.parseInt();

  // Empty serial port
  for (int n=0; n<Serial.available(); n++)
  {
    Serial.read();
  }

  // Determine which motor to move
  int motor_select = 1000;  // Units to select motor and determine movement
  int motor = floor(command/motor_select);
  float movement = command % motor_select;
  String motor_name;

  switch (motor)
  {
  case 1:
    inclineActuator.moveToPercentage(movement);
    motor_name = "Incline actuator";
    break;

  case 2:
    nema8.moveRun(movement);
    motor_name = "NEMA8";
    break;

  default:
    break;
  }

  Serial.println("Movement request: ");
  Serial.println("- Motor: " + motor_name);
  Serial.println("- Movement: " + String(movement));

  Serial.println("\nSelect motor and movement...");
}
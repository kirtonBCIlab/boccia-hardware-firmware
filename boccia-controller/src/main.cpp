// Include libraries
#include <Arduino.h>
#include <BocciaStepper.h>
#include <LinearActuator.h>
#include <Functions.h>
#include <AccelStepper.h>

int n_steps = 200;
int dir = 1;

// Change number of pins
// Define incline actuator
int incline_pin1 = 7;
int incline_pin2 = 6;
int incline_pin_pot = 0;
int incline_speed_threshold = 15;
int incline_speed_factor = 50;
int incline_pin_sensor = 18;
// LinearActuator inclineActuator(pin1, pin2, pin_sensor, speed_threshold, speed_factor);
// If pin sensor is enabled, the calibration depends on the 
LinearActuator inclineActuator(incline_pin1, incline_pin2, incline_pin_pot, incline_speed_threshold, incline_speed_factor, incline_pin_sensor);

// Define elevator actuator
int elevator_pin1 = 5;
int elevator_pin2 = 4;
int elevator_pin_pot = 1;
int elevator_speed_threshold = 15;
int elevator_speed_factor = 50;
// LinearActuator elevatorActuator(pin1, pin2, pin_sensor, speed_threshold, speed_factor);
// If pin sensor is enabled, the calibration depends on the 
LinearActuator elevatorActuator(elevator_pin1, elevator_pin2, elevator_pin_pot, elevator_speed_threshold, elevator_speed_factor);

// Define Nema8
int pin_step = 52;
int pin_dir = 53;
BocciaStepper nema8(AccelStepper::DRIVER, pin_step, pin_dir);

// Prototype functions
void nema8Limit();
void inclineLimit();
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

  // Calibration steps
  Serial.println("Calibration");
  Serial.println("NEMA17 - Calibration started");
  nema8.findRange();
  Serial.println("NEMA8 - Calibration ended");

  Serial.println("NEMA23 - Calibration started");
  nema24.findRange();
  Serial.println("NEMA24 - Calibration ended");

  Serial.println("Incline actuator - Calibration started");
  inclineActuator.findRange();
  Serial.println("Calibration ended");

  // Add elevator motor

  Serial.println("\nSelect motor and movement...");
}

void loop() 
{
  if (Serial.available())
  {
    decodeCommand();
  }

  waitMillis(250);
}


void nema8Limit()
{
  nema8.limitDetected();
}

void decodeCommand()
{
  // Add documentation on how this works

  long command = Serial.parseInt();

  // Empty serial port
  for (int n=0; n<Serial.available(); n++)
  {
    Serial.read();
  }

  // Determine which motor to move
  int motor_select = 1000;  // Units to select motor and determine movement
  int motor = abs(floor(command/motor_select));
  int movement = command % motor_select;
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

   case 4:
    elevatorActuator.moveToPercentage(movement);
    motor_name = "Elevator Actuator";
    break;

  case 4:
    // Implement elevator actuator

  case 9:
    // Implement find range with first two digits
    // 9100 : Calibrate motor  case 1

  default:
    Serial.println("Wrong command: " + String(command));
    break;
  }

  Serial.println("\nCommand received: " + String(command));
  Serial.println("Movement request: ");
  Serial.println("- Motor: " + motor_name);
  Serial.println("- Movement: " + String(movement));

  Serial.println("\nSelect motor and movement...");
}
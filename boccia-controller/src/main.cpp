// Include libraries
#include <Arduino.h>
#include <BocciaStepper.h>
#include <LinearActuator.h>
#include <Functions.h>
#include <AccelStepper.h>

int n_steps = 200;
int dir = 1;

// Define incline actuator
int incline_pin1 = 7;
int incline_pin2 = 6;
int incline_pin_pot = 0;
int incline_speed_threshold = 15;
int incline_speed_factor = 50;
int incline_pin_sensor = 18;      // If pin sensor is enabled (i.e., !0), the calibration depends on the pin sensor trigger
LinearActuator inclineActuator(incline_pin1, incline_pin2, incline_pin_pot, incline_speed_threshold, incline_speed_factor, incline_pin_sensor);

// Define elevator actuator
int elevator_pin1 = 5;
int elevator_pin2 = 4;
int elevator_pin_pot = 1;
int elevator_speed_threshold = 15;
int elevator_speed_factor = 50;
LinearActuator elevatorActuator(elevator_pin1, elevator_pin2, elevator_pin_pot, elevator_speed_threshold, elevator_speed_factor);

// Define nema17
int pin_step = 52;
int pin_dir = 53;
BocciaStepper nema17(AccelStepper::DRIVER, pin_step, pin_dir);

// Define nema23
int nema23_pin_step = 50;
int nema23_pin_dir = 51;
BocciaStepper nema23(AccelStepper::DRIVER, nema23_pin_step, nema23_pin_dir);

// Prototype functions
void nema17Limit();
void nema23Limit();
void waitMillis(unsigned long wait_msec);
void decodeCommand();

void setup() {
  // Serial communication debugging
  Serial.begin(9600);
  Serial.println("Begin setup");

  // Nema17 settings
  nema17.setReturnSteps(10);
  nema17.setDefaultSpeed(400);   // Default speed [steps/sec]
  nema17.setDefaultAccel(10);
  nema17.setMaxSpeed(1000);    // Maximum speed [steps/sec]
  nema17.setInterruptPin(2);
  nema17.setNoSteps(200);      // Number of steps for complete rotation [steps]

  // Nema23 settings
  nema23.setReturnSteps(10);
  nema23.setDefaultSpeed(400);   // Default speed [steps/sec]
  nema23.setDefaultAccel(10);
  nema23.setMaxSpeed(1000);    // Maximum speed [steps/sec]
  nema23.setInterruptPin(3);
  nema23.setNoSteps(800);      // Number of steps for complete rotation [steps]

  // Set NEMA inputs to ground
  digitalWrite(pin_dir, 0);
  digitalWrite(pin_step, 0);

  // Interrupts
  attachInterrupt(digitalPinToInterrupt(nema17.getInterruptPin()), nema17Limit, RISING);
  attachInterrupt(digitalPinToInterrupt(nema23.getInterruptPin()), nema23Limit, RISING);

  Serial.println("Calibration");
  // Serial.println("nema17 - Calibration started");
  // nema17.findRange();
  // Serial.println("nema17 - Calibration ended");

  // Serial.println("nema23 - Calibration started");
  // nema23.findRange();
  // Serial.println("nema23 - Calibration ended");

  Serial.println("Calibrating - nema17");
  nema17.findRange();
  Serial.println("nema17 - Calibration ended");

  Serial.println("Calibrating -  incline actuator");
  inclineActuator.findRange();
  Serial.println("Calibration ended");

  Serial.println("Calibrating -  elevator actuator");
  elevatorActuator.findRange();
  Serial.println("Calibration ended");

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


void nema17Limit()
{
  nema17.limitDetected();
}

void nema23Limit()
{
  nema23.limitDetected();
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
    nema17.moveRun(movement);
    motor_name = "nema17";
    break;

  case 3:
    nema23.moveRun(movement);
    motor_name = "nema23";
    break;
   
   case 4:
    elevatorActuator.moveToPercentage(movement);
    motor_name = "Elevator Actuator";
    break;

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
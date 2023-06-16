// Include libraries
#include <Arduino.h>
#include <BocciaStepper.h>
#include <LinearActuator.h>
#include <Functions.h>
#include <AccelStepper.h>

int n_steps = 200;
int dir = 1;

// NEMA 8 motor for testing purposes
int nema8_pin_step = 3;
int nema8_pin_dir = 6;
BocciaStepper nema8(AccelStepper::DRIVER, nema8_pin_step, nema8_pin_dir);

// Build motor objects
// - Nema 17
int nema17_pin_step = 5;
int nema17_pin_dir = 6;
BocciaStepper nema17(AccelStepper::DRIVER, nema17_pin_step, nema17_pin_dir);

// - Nema 23
int nema23_pin_step = 12;
int nema23_pin_dir = 11;
BocciaStepper nema23(AccelStepper::DRIVER, nema23_pin_step, nema23_pin_dir);

// - Incline actuator
int incline_pin1 = 7;
int incline_pin2 = 8;           
int incline_pin_pot = 4;          // Analog pin for potentiometer
int incline_speed_threshold = 15;
int incline_speed_factor = 50;
int incline_pin_sensor = 0;       // If pin sensor is enabled (i.e., !0), the calibration depends on the pin sensor trigger
LinearActuator inclineActuator(incline_pin1, incline_pin2, incline_pin_pot, incline_speed_threshold, incline_speed_factor, incline_pin_sensor);

// - Elevator actuator
int elevator_pin1 = 9;
int elevator_pin2 = 10;
int elevator_pin_pot = 3;
int elevator_speed_threshold = 15;
int elevator_speed_factor = 50;
LinearActuator elevatorActuator(elevator_pin1, elevator_pin2, elevator_pin_pot, elevator_speed_threshold, elevator_speed_factor);

// Prototype functions
void nema17Limit();
void nema23Limit();
void waitMillis(unsigned long wait_msec);
void decodeCommand();

void setup() {
  // Serial communication debugging
  Serial.begin(9600);
  Serial.println("Begin setup");

  // Set motor settings
  // - Nema 17
  nema17.setReturnSteps(10);
  nema17.setDefaultSpeed(400);      // Default speed [steps/sec]
  nema17.setDefaultAccel(10);
  nema17.setMaxSpeed(1000);         // Maximum speed [steps/sec]
  nema17.setInterruptPin(2);
  nema17.setNoSteps(200);           // Number of steps for complete rotation [steps]
  digitalWrite(nema17_pin_dir, 0);  // Set pins to ground to avoid that initial jump
  digitalWrite(nema17_pin_step, 0);

  // - Nema 23
  nema23.setReturnSteps(10);
  nema23.setDefaultSpeed(400);  // Default speed [steps/sec]
  nema23.setDefaultAccel(10);
  nema23.setMaxSpeed(1000);     // Maximum speed [steps/sec]
  nema23.setInterruptPin(3);
  nema23.setNoSteps(800);       // Number of steps for complete rotation [steps]

  // Interrupts
  attachInterrupt(digitalPinToInterrupt(nema17.getInterruptPin()), nema17Limit, RISING);
  attachInterrupt(digitalPinToInterrupt(nema23.getInterruptPin()), nema23Limit, RISING);

  // Calibration steps - Enable sections as needed
  Serial.println("Calibration");

  // - Nema 17
  // Serial.println("Nema 17 - Calibration started");
  // nema17.findRange();
  // Serial.println("Nema 17 - Calibration ended");
 
  // - Nema 23
  // Serial.println("Nema 23 - Calibration started");
  // nema23.findRange();
  // Serial.println("Nema 23 - Calibration ended");

  // - Incline actuator
  // Serial.println("Incline - Calibration started");
  // inclineActuator.findRange();
  // Serial.println("Incline - Calibration ended");

  //  - Elevator actuator
  Serial.println("Elevator - Calibration started");
  elevatorActuator.findRange();
  Serial.println("Elevator - Calibration ended");

  Serial.println("\nSelect motor and movement...");
}

void loop() 
{
  if (Serial.available())
  {
    decodeCommand();
  }

  waitMillis(250);  // Wait a bit while decoding command
}


void nema17Limit()
{
  nema17.limitDetected();
}

void nema23Limit()
{
  nema23.limitDetected();
}

/// @brief The input command must be a positive or negative number with 
/// 4 digits. The first digit selects the motor according to the switch
/// case function inside this function. The other three digits select 
/// the number of positive or negative steps for the stepper motors, or
/// the percentage of the linear actuator (where 0\% is retracted, and
/// 100\% is extended). 
/// 
/// If the number starts with "9", the second digit selects the motor
/// to re-do the calibration. 
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
  int gross_motor_select = 100000;  // Units to select gross movement
  int motor = abs(floor(command/motor_select));
  int movement = command % motor_select;
  String motor_name;

  // Check if the command includes gross movement
  if (command >= gross_motor_select) {
        int gross_move = abs(floor(command/gross_motor_select));
        int gross_rotation = gross_move % 10;
        int gross_elevation = gross_move / 10;

        // Handle rotation(assuming 1.8 degree per step)
        switch(gross_rotation) {
            case 1: nema23.moveRun(-22); break;
            case 2: nema23.moveRun(-11); break;
            case 3: nema23.moveRun(0); break;
            case 4: nema23.moveRun(11); break;
            case 5: nema23.moveRun(22); break;
            default: Serial.println("Invalid gross rotation command"); break;
        }

        // Handle elevation
        switch(gross_elevation) {
            case 1: elevatorActuator.moveToPercentage(20); break;
            case 2: elevatorActuator.moveToPercentage(40); break;
            case 3: elevatorActuator.moveToPercentage(60); break;
            case 4: elevatorActuator.moveToPercentage(80); break;
            case 5: elevatorActuator.moveToPercentage(100); break;
            default: Serial.println("Invalid gross elevation command"); break;
        }

        // use motor and movement for the regular command
        motor = command % gross_motor_select / motor_select;
        movement = command % motor_select;
    }


  switch (motor)
  {
  case 1:
    nema17.moveRun(movement);
    motor_name = "nema17";

  case 2:
    nema23.moveRun(movement);
    motor_name = "nema23";
  
  case 3:
    inclineActuator.moveToPercentage(movement);
    motor_name = "Incline actuator";
    break;

   case 4:
    elevatorActuator.moveToPercentage(movement);
    motor_name = "Elevator Actuator";
    break;

  case 9:
    int motor_calibration = abs(floor(movement/100));
    
    switch (motor_calibration)
    {
    case 1: nema17.findRange(); break;   
    case 2: nema23.findRange(); break;
    case 3: inclineActuator.findRange(); break;
    case 4: elevatorActuator.findRange(); break;
    default: Serial.println("Incorrect command to calibrate"); break;
    }
    
    Serial.print("Recalibrating: " + String(motor_name));

  default:
    Serial.println("Incorrect command: " + String(command));
    break;
  }

  Serial.println("\nCommand received: " + String(command));
  Serial.println("Movement request: ");
  Serial.println("- Motor: " + motor_name);
  Serial.println("- Movement: " + String(movement));

  Serial.println("\nSelect motor and movement...");
}
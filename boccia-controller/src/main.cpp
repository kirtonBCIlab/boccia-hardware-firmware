// Include libraries
#include <Arduino.h>
#include <BocciaStepper.h>
#include <LinearActuator.h>
#include <Functions.h>
#include <AccelStepper.h>

// Build motor objects
// - Release
int release_pin_step = 5;
int release_pin_dir = 6;
int release_interrupt_pins[2] = {19,0};
int release_nsteps = 200;
int release_nsteps_return = 10;
int release_default_speed = 400;
BocciaStepper release(release_pin_step, release_pin_dir, release_interrupt_pins, release_nsteps, release_nsteps_return, release_default_speed);

// - Rotation
int rotation_pin_step = 12;
int rotation_pin_dir = 11;
int rotation_interrupt_pins[2] = {3,0};
int rotation_nsteps = 200;
int rotation_nsteps_return = 10;
BocciaStepper rotation(rotation_pin_step, rotation_pin_dir, rotation_interrupt_pins, rotation_nsteps, rotation_nsteps_return);

// - Incline actuator
int incline_pin1 = 7;
int incline_pin2 = 8;           
int incline_pin_pot = 4;          // Analog pin for potentiometer
int incline_speed_threshold = 15;
int incline_speed_factor = 50;
int incline_pin_sensor = 0;       // If pin sensor is enabled (i.e., !0), the calibration depends on the pin sensor trigger
LinearActuator incline(incline_pin1, incline_pin2, incline_pin_pot, incline_speed_threshold, incline_speed_factor, incline_pin_sensor);

// - Elevator actuator
int elevator_pin1 = 9;
int elevator_pin2 = 10;
int elevator_pin_pot = 3;
int elevator_speed_threshold = 15;
int elevator_speed_factor = 50;
LinearActuator elevation(elevator_pin1, elevator_pin2, elevator_pin_pot, elevator_speed_threshold, elevator_speed_factor);

// Prototype functions
void releaseLimit();
void rotationLimit();
void waitMillis(unsigned long wait_msec);
void decodeCommand();

void setup() {
  // Serial communication debugging
  Serial.begin(9600);
  Serial.println("Begin setup");

  // Initialize motors
  // release.initializePins();
  // rotation.initializePins();
  // incline.initializePins();
  // elevation.initializePins();
  
  // Interrupts
  attachInterrupt(digitalPinToInterrupt(release_interrupt_pins[0]), releaseLimit, RISING);
//  attachInterrupt(digitalPinToInterrupt(rotation_interrupt_pins[0]), rotationLimit, RISING);

  // Calibration steps - Enable sections as needed
  // Serial.println("Calibration");

  // // - Release
  // Serial.println("Release - Calibration started");
  // release.findRange();
  // Serial.println("Release - Calibration ended");
 
  // // - Rotation
  // Serial.println("Rotation - Calibration started");
  // rotation.findRange();
  // Serial.println("Rotation - Calibration ended");

  // // - Incline actuator
  // Serial.println("Incline - Calibration started");
  // incline.findRange();
  // Serial.println("Incline - Calibration ended");

  // //  - Elevator actuator
  // Serial.println("Elevator - Calibration started");
  // elevation.findRange();
  // Serial.println("Elevator - Calibration ended");

  // Serial.println("\nSelect motor and movement...");
}

void loop() 
{
  if (Serial.available()) { decodeCommand(); }
  waitMillis(250);  // Wait a bit while decoding command
}

void releaseLimit()
{
  release.stopDetected();
}

void rotationLimit()
{
  rotation.limitDetected();
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
  for (int n=0; n<Serial.available(); n++) { Serial.read(); }

  // Determine which motor to move
  String motor_names[4] = {"release", "rotation", "incline", "elevation"};
  int motor_select = 1000;  // Units to select motor and determine movement
  // int gross_motor_select = 100000;  // Units to select gross movement
  int motor = abs(floor(command/motor_select)); 
  int movement = command % motor_select;
  
  
  String motor_name = motor_names[motor-1];
  Serial.println("Selected motor " + motor_name + " - " + String(motor));

  // Check if the command includes gross movement
  // if (command >= gross_motor_select) {
  //       int gross_move = abs(floor(command/gross_motor_select));
  //       int gross_rotation = gross_move % 10;
  //       int gross_elevation = gross_move / 10;

  //       // Handle rotation(assuming 1.8 degree per step)
  //       switch(gross_rotation) {
  //           case 1: rotation.moveRun(-22); break;
  //           case 2: rotation.moveRun(-11); break;
  //           case 3: rotation.moveRun(0); break;
  //           case 4: rotation.moveRun(11); break;
  //           case 5: rotation.moveRun(22); break;
  //           default: Serial.println("Invalid gross rotation command"); break;
  //       }

  //       // Handle elevation
  //       switch(gross_elevation) {
  //           case 1: elevation.moveToPercentage(20); break;
  //           case 2: elevation.moveToPercentage(40); break;
  //           case 3: elevation.moveToPercentage(60); break;
  //           case 4: elevation.moveToPercentage(80); break;
  //           case 5: elevation.moveToPercentage(100); break;
  //           default: Serial.println("Invalid gross elevation command"); break;
  //       }

  //       // use motor and movement for the regular command
  //       motor = command % gross_motor_select / motor_select;
  //       movement = command % motor_select;
  //   }

  switch (motor)
  {
  case 1: release.releaseBall(movement);        break;
  case 2: rotation.moveRun(movement);           break;  
  case 3: incline.moveToPercentage(movement);   break;
  case 4: elevation.moveToPercentage(movement); break;

  case 9:
  {
    int motor_calibration = abs(floor(movement/100));
    motor_name = motor_names[motor_calibration-1];
    Serial.println("Recalibrating: " + String(motor_name));

    switch (motor_calibration)
    {
    case 1: release.findRange(); break;   
    case 2: rotation.findRange(); break;
    case 3: incline.findRange(); break;
    case 4: elevation.findRange(); break;
    default: Serial.println("Incorrect command to calibrate"); break;
    }
    break;
  }    

  default:
    Serial.println("Incorrect command: " + String(command));
    break;
  }

  if (motor != 9)
  {
    Serial.println("\nCommand received: " + String(command));
    Serial.println("Movement request: ");
    Serial.println("- Motor: " + motor_name);
    Serial.println("- Movement: " + String(movement));

    Serial.println("\nSelect motor and movement...");
  }

}
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
int release_interrupt_pins[2] = {0,2};
int release_nsteps = 800;
int release_nsteps_return = 15;
int release_default_speed = 600;
int release_default_accel = 30;
bool release_use_limits = false;
BocciaStepper release(
  release_pin_step,
  release_pin_dir,
  release_interrupt_pins,
  release_nsteps,
  release_nsteps_return,
  release_default_speed,
  release_default_accel,
  release_use_limits);

// - Rotation
int rotation_pin_step = 12;
int rotation_pin_dir = 11;
int rotation_interrupt_pins[2] = {3,19};
int rotation_nsteps = 800;
int rotation_nsteps_return = 180;
int rotation_default_speed = 600;
int rotation_default_accel = 30;
bool rotation_use_limits = true;
int rotation_gear_ratio = 3;
BocciaStepper rotation(
  rotation_pin_step,
  rotation_pin_dir,
  rotation_interrupt_pins,
  rotation_nsteps,
  rotation_nsteps_return,
  rotation_default_speed,
  rotation_default_accel,
  rotation_use_limits,
  rotation_gear_ratio);
 
// - Incline actuator
int incline_pin1 = 8;
int incline_pin2 = 7;           
int incline_pin_pot = 4;          // Analog pin for potentiometer
int incline_speed_threshold = 15;
int incline_speed_factor = 50;
int incline_pin_sensor = 7;       // If pin sensor is enabled (i.e., !0),
                                  // the calibration depends on the pin sensor trigger
int incline_pin_threshold = 600;  
LinearActuator incline(
  incline_pin1,
  incline_pin2,
  incline_pin_pot,
  incline_speed_threshold,
  incline_speed_factor,
  incline_pin_sensor,
  incline_pin_threshold);

// - Elevator actuator
int elevator_pin1 = 9;
int elevator_pin2 = 10;
int elevator_pin_pot = 3;
int elevator_speed_threshold = 15;
int elevator_speed_factor = 50;
int elevator_manual_limits[2] = {20, 360};  // ADC values found manually
                                            // Use them with presetRange()
LinearActuator elevation(
  elevator_pin1,
  elevator_pin2,
  elevator_pin_pot,
  elevator_speed_threshold,
  elevator_speed_factor);

// Prototype functions
void releaseLimit();
void leftLimit();
void rightLimit();
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

  // Check that sensors are cleared
  release.clearSensorWhileStopped(release_interrupt_pins[1]);
  rotation.clearSensorWhileStopped(rotation_interrupt_pins[0]);
  rotation.clearSensorWhileStopped(rotation_interrupt_pins[1]);
  
  // Interrupts
  attachInterrupt(digitalPinToInterrupt(release_interrupt_pins[1]), releaseLimit, RISING);
  attachInterrupt(digitalPinToInterrupt(rotation_interrupt_pins[0]), leftLimit, RISING);
  attachInterrupt(digitalPinToInterrupt(rotation_interrupt_pins[1]), rightLimit, RISING);

  // Calibration steps - Enable sections as needed
  Serial.println("Calibration");

  // - Release
  // Serial.println("Release - Starting position started");
  // release.moveRun(-release_nsteps);
  // Serial.println("Release - Starting position ended");
 
  // - Rotation
  // Serial.println("Rotation - Calibration started");
  // rotation.findRange();
  // Serial.println("Rotation - Calibration ended");

  // - Incline actuator
  // Serial.println("Incline - Calibration started");
  // incline.findRange();
  // Serial.println("Incline - Calibration ended");

  // - Elevator actuator
  // -- Select either findRange() or presetRange()
  // Serial.println("Elevator - Calibration started");
  // elevation.findRange();
  // elevation.presetRange(elevator_manual_limits[1], elevator_manual_limits[2]);
  // Serial.println("Elevator - Calibration ended");

  Serial.println("\nSelect motor and movement...");
}

void loop() 
{
  if (Serial.available()) { decodeCommand(); }
  waitMillis(250);  // Wait a bit while decoding command
}

void releaseLimit()
{
  release.active_interrupt_pin = release_interrupt_pins[0];
  release.limitDetected();
}

void leftLimit()
{
  rotation.limitDetected();
  rotation.active_interrupt_pin = rotation_interrupt_pins[0];
}

void rightLimit()
{
  rotation.limitDetected();
  rotation.active_interrupt_pin = rotation_interrupt_pins[1];
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

  if (command>9999)
  {
  int gross_motor_select = 1000;  // Determines the boundary between rotation and elevation
  int gross_elevation = command % gross_motor_select;
  int gross_rotation = floor(command / gross_motor_select);

  
  elevation.moveToPercentage(gross_elevation);
  rotation.moveDegrees(gross_rotation);

  Serial.println("Gross movement command received: " + String(command));
  Serial.println("Movement request: ");
  Serial.println("- Gross Rotation : " + String(gross_rotation));
  Serial.println("- Gross Elevation : " + String(gross_elevation));
  }

  else

  {

  // Determine which motor to move
  String motor_names[4] = {"release", "rotation", "incline", "elevation"};
  int motor_select = 1000;  // Units to select motor and determine movement
  int motor = abs(floor(command/motor_select)); 
  Serial.println("Case: " + String(motor));
  int movement = command % motor_select;
  
  
  String motor_name = motor_names[motor-1];
  switch (motor)
  {
    case 1: release.releaseBall(movement);        break;
    case 2: rotation.moveDegrees(movement);       break;  
    case 3: incline.moveToPercentage(movement);   break;
    case 4: elevation.moveToPercentageRange(movement); break;

    case 9:
    {
      int motor_calibration = abs(floor(movement/100));
      motor_name = motor_names[motor_calibration-1];
      Serial.println("Recalibrating: " + String(motor_name));

      switch (motor_calibration)
      {
        case 1: release.moveDegrees(release_nsteps); break;   
        case 2: rotation.findRange(); break;
        case 3: incline.findRange(); break;
        case 4: elevation.findRange(); break;
        case 5: elevation.presetRange(elevator_manual_limits[1], elevator_manual_limits[2]); break;
        default: Serial.println("Incorrect command to calibrate");
      }
    } break;   

  default:
    Serial.println("Incorrect command: " + String(command));
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
}
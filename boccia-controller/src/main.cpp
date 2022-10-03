// Include libraries
#include <Arduino.h>
#include <BocciaStepper.h>
#include <LinearStepper.h>
#include <Functions.h>

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
// void nema8Limit();
// void waitMillis(unsigned long wait_msec);

void setup() {
  // Serial communication debugging
  Serial.begin(9600);
  Serial.println("Begin setup");

  // Incline actuator settings

  // Nema 8 settings
  nema8.setNoSteps(200);
  nema8.setReturnSteps(10);
  nema8.setDefaultSpeed(400);   // Default speed [steps/sec]
  nema8.setDefaultAccel(10);
  nema8.setMaxSpeed(1000);    // Maximum speed [steps/sec]
  nema8.setInterruptPin(2);
  nema8.setNoSteps(200);      // Number of steps for complete rotation [steps]

  // Interrupts
  // attachInterrupt(digitalPinToInterrupt(nema8.getInterruptPin()), nema8Limit, RISING);

  // Serial.println("Begin calibration");
  // nema8.findRange();
  // inclineActuator.findRange();
  // Serial.println("End calibration\nInput steps to move...");

}

void loop() 
{
  inclineActuator.findRange();

  // if (Serial.available())
  // {
  //   n_steps = Serial.parseInt();
    
  //   // Empty serial port
  //   for (int n=0; n<Serial.available(); n++)
  //   {
  //     Serial.read();
  //   }
    
  //   Serial.println("Current position: " + String(nema8.currentPosition()));
  //   Serial.println("Moving " + String(n_steps) + " steps");
  //   nema8.moveRun(n_steps);

  // }
  // waitMillis(500);
}

// void nema8Limit()
// {
//   nema8.limitDetected();
// }

// void waitMillis(unsigned long wait_msec)
// {
//     unsigned long deltaT, t0, t1 = 0;
//     t0 = millis();
    
//     do
//     {
//       t1 = millis();
//       deltaT = t1-t0;
//     }while(deltaT < wait_msec);
// }
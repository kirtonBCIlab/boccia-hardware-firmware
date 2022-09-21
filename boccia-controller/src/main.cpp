// Include libraries
#include <Arduino.h>
#include <BocciaStepper.h>

int n_steps = 200;
int dir = 1;

// Define Nema8
int pin_step = 53;
int pin_dir = 52;
BocciaStepper nema8(AccelStepper::DRIVER, pin_step, pin_dir);

// Prototype functions
void nema8Limit();
void waitMillis(unsigned long wait_msec);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Begin setup");
  nema8.setMaxSpeed(1000);    // Maximum speed [steps/sec]
  nema8.setAcceleration(10);  // Acceleration [steps/sec^2]
  nema8.setSpeed(400);        // Speed [steps/sec]
  nema8.setInterruptPin(2);

  // Interrupts
  attachInterrupt(digitalPinToInterrupt(nema8.getInterruptPin()), nema8Limit, RISING);
}

void loop() 
{
  nema8.moveRun(n_steps*dir);
  waitMillis(500);
  dir = dir * -1;
}

void nema8Limit()
{
  nema8.limitDetected();
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
#include "BocciaStepper.h"
#include <AccelStepper.h>
  
  void BocciaStepper::moveRun(long relative)
    {
      Serial.println("You are here");
      AccelStepper::move(relative);
      Serial.println("Distance to go: " + String(AccelStepper::distanceToGo()));
      while (AccelStepper::distanceToGo() != 0)
      {
        AccelStepper::run();
      }
    }

  void BocciaStepper::setInterruptPin(int pin_sensor)
  {
    pin_interrupt = pin_sensor;
    pinMode(pin_interrupt, INPUT);
    
    Serial.println("Interrupt pin set: " + String(pin_interrupt));  
  }

  int BocciaStepper::getInterruptPin()
  {
    return pin_interrupt;
  }

  void BocciaStepper::limitDetected()
  {
    Serial.println("Limit detected");
    int step_direction;
    
    Serial.println("Stopping motor and turning back");
    Serial.println("Target: " + String(targetPosition()));
    Serial.println("Current; " +String(currentPosition()));
    if (targetPosition() - currentPosition() < 0)
    { step_direction = 1; }
    else
    { step_direction = -1; }

    int step_current = currentPosition();
    stop();
    setCurrentPosition(0);
    move(5 * step_direction);
  }

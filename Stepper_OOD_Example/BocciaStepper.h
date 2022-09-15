#ifndef BOCCIASTEPPER_H
#define BOCCIASTEPPER_H
#include <AccelStepper.h>
#include <Arduino.h>

class BocciaStepper:public AccelStepper
{
  private:
    int pin_interrupt;
    bool homing_flag;
  
  using AccelStepper::AccelStepper;
  
  public:
    void moveRun(long relative);
    void setInterruptPin(int pin_sensor);
    int getInterruptPin();
    void limitDetected();
};


#endif

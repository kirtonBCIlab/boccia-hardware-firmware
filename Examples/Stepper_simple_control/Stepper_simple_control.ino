/*
 * Stepper simple control
 * - This script takes in a Serial command to set the position of the Stepper motor
 * - Positive numbers are clockwise steps, negative numbers are counter clockwise steps
 */

// Include the AccelStepper Library
#include <AccelStepper.h>

// Define pin numbers
const int pin_dir = 3;
const int pin_step = 2;

// Create AccelStepper object
AccelStepper nema8(AccelStepper::DRIVER, pin_step, pin_dir);

void setup() {
  // Set motor parameters
  nema8.setMaxSpeed(1000);    // Speed [steps/sec]
  nema8.setAcceleration(10);  // Acceleration [steps/sec^2]
                              // High values will cause skipping steps
  nema8.setSpeed(400);

  Serial.begin(9600);
  Serial.println("Input number of steps to move");
}

void loop() {  
  if(Serial.available() > 0)
  {
    waitMillis(200); // Wait to get the whole message
    long pos = Serial.parseInt();

    // Make sure nothing stays in the buffer
//    for(int i=0; i<=Serial.available()-1; i++)
//    {
//      Serial.read();
//    }
    
    Serial.println("Received, move: " + String(pos));

    nema8.move(pos);
    waitMillis(100);
    Serial.println("Data in buffer: " + String(Serial.available()));
    Serial.println(" Start movement");

    do
    {
      nema8.run();
    }while(nema8.distanceToGo() != 0);
    Serial.println(" Movement finished\nWaiting for next command...\n");
  }
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

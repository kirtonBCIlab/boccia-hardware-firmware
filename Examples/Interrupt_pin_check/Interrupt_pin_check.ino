/*
 * Interrupt read test
 * - Use this test to activate the multiple interrupts in the ramp
 *   and detect which sensor is connected to each pin
 */

int pins[3] = {2,3,19}; // Pin numbers associated with interrupts

// Prototype functions
void releaseInterrupt();
void rotationLeftInterrupt();
void rotationRightInterrupt();

void setup()
{
  Serial.begin(9600);
  Serial.println("Begin program");
  
  // Set all pins as inputs
  for (int i=0; i<3; i++) { pinMode(pins[i], INPUT); }

  attachInterrupt(pins[0], releaseInterrupt, RISING);
  attachInterrupt(pins[1], rotationLeftInterrupt, RISING);
  attachInterrupt(pins[2], rotationRightInterrupt, RISING);
}

void loop() 
{
  // put your main code here, to run repeatedly:
  int a = 0;
}

//void releaseInterrupt() { Serial.println("I am release: " + String(pins[0])); }
//void rotationLeftInterrupt() { Serial.println("I am left: " + String(pins[1])); }
//void rotationRightInterrupt() { Serial.println("I am right: " + String(pins[2])); }

void releaseInterrupt() { Serial.print(pins[0]); }
void rotationLeftInterrupt() { Serial.print(pins[1]); }
void rotationRightInterrupt() { Serial.print(pins[2]); }

/*
  Actuator simple control
  - Test code to move the actuator forward or in reveser through Serial communication
*/

// Setup variables
int driver1 = 7;  // Pin connected to IN1
int driver2 = 8;  // Pin connected to IN2
int pot_pin = A7;
int pot_val;
String command;   // Command to be sent through serial 

void setup()
{
  Serial.begin(9600); // Start serial connection, select baud rate
  Serial.println("Write the command: extend (ext) or retract (ret)");
  pinMode(driver1, OUTPUT);
  pinMode(driver2, OUTPUT);

  // Begin with drivers on hold
  digitalWrite(driver1, LOW);
  digitalWrite(driver2, LOW);
}

void loop()
{
  if (Serial.available()>0)
  {
    command = Serial.readStringUntil('\n');
    command.trim();

    Serial.println("\nCommand received: " + command);
    move_motor(command); 
  }
}

void move_motor(String command)
{
  // Move motor
  if (command == "ext")
  {
    digitalWrite(driver1, HIGH);
    digitalWrite(driver2, LOW);
  }
  else if (command == "ret")
  {
    digitalWrite(driver1, LOW);
    digitalWrite(driver2, HIGH);
  }
  else
  {
    Serial.println("Command not recognized \n");
  }

  waitTime(500); 

  // Hold position
  digitalWrite(driver1, LOW);
  digitalWrite(driver2, LOW);

  // Print value of potentiometer 
  pot_val = analogRead(pot_pin);
  Serial.println("Pot val: " + String(pot_val));
}

void waitTime(unsigned long wait_msec)
{
    unsigned long deltaT, t0, t1 = 0;
    t0 = millis();
    
    do
    {
      t1 = millis();
      deltaT = t1-t0;
    }while(deltaT < wait_msec);

    Serial.println("Wait time: " + String(deltaT) + " msec");
}

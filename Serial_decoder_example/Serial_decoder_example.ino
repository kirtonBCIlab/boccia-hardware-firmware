/*
  Serial Decoder Example
  - This script takes in a serial command with a preset number of characters
  - The serial command is then stored in a char array and displayed over serial communication 

  Notes;
  - Change the num_char variable to the length of the desired command
 */


const int num_char = 6; // Size of array of characters of expected serial command
char command[num_char]; // Command sent over serial

void setup() 
{
  Serial.begin(9600);
  Serial.println("Program started");
}

void loop() 
{
  if (Serial.available()>0)
  {
    // Wait X msec
    waitTime(100);

    // Decode message sent over serial
    Serial.println("Begin decoding");    
    decodeCommand(num_char);

    Serial.println("Print command char array");
    for (int i=0; i<num_char; i++)
    {
      Serial.println(command[i]);
    }
  }  
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

    Serial.println("Wait time: " + String(deltaT));
}

void decodeCommand(int num_bytes)
{
  Serial.println("How many char? " + String(num_bytes));
  char c;

  for (int index=0; index<=num_bytes; index++)
  {
    if (index == num_bytes)
    {
      c = Serial.read();
      Serial.println("Command finished");
      Serial.println("Complete command: " + String(command));
    }
    else
    {   
      c = Serial.read();
      command[index] = c;
      Serial.println("Index: " + String(index));
      Serial.println("Serial read: " + String(c));
    }
  }
}

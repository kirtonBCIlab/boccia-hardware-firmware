/*
    Test code to receive serial data in Arduino and toggle the status of an LED (Serial communication in Arduino 3)
    For more details, visit: https://techzeero.com/arduino-tutorials/serial-communication-in-arduino/
*/

const int green_LED = 4;  // Digital pin for green LED  
const int red_LED = 5;    // Digital pin for red LED
int led_state = LOW;      // Current state of LED
int blinkRate = 500;      // Delay time [msec]

void setup()
{
  Serial.begin(9600); // Start serial connection, select baud rate
  pinMode(green_LED, OUTPUT);
  pinMode(red_LED, OUTPUT);
  delay(500);
  Serial.println("Communication started");
}

// Switch LED state
bool LED_switch(int led)
{
  led_state = digitalRead(led);
  //led_state = !led_state;
  digitalWrite(led, !led_state);
  return !led_state;
}

void loop()
{
  if (Serial.available()>0)
  {
    char ch = Serial.read();
    
    switch (ch)
    {
      case '4':
        bool green_LED_state = LED_switch(green_LED);
        Serial.println("Green LED:" + String(green_LED_state));
        break;

      case '5':
        bool red_LED_state = LED_switch(red_LED);
        Serial.println("Green LED:" + String(red_LED_state));
        break;          
      }

    String statement = "Read: ";
    Serial.println(statement+ch);
  }

 
}
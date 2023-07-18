/*
 * Analog read test
 * - This function reads an analog value from the desired pin and 
 *   displays the value in the Serial monitor
 */

int analog_pin = 7;

void setup() 
{
  Serial.begin(9600);
  pinMode(analog_pin, INPUT);
}

void loop() 
{
  int analog_read = analogRead(analog_pin);
  Serial.print(analog_read);
  Serial.println("");
}

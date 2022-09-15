/*
  Serial test Mega
  - Test script to try serial communication with Arduino MEGA
  - Serial communication:
  -- i = Built-in LED turns on
  -- o = Built-in LED turns off
*/

String serial_command;

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.write("Hello world");
}

void loop() {
  // if (Serial.available()>0)
  // {
  //   Serial.println("Hello world");
  //   digitalWrite(LED_BUILTIN, LOW);
  // }
  
  if (Serial.available() > 0)
  {
    serial_command = Serial.readStringUntil('\n');
    serial_command.trim();
    
    Serial.println("Received: " + serial_command);
    delay(200);

    if (serial_command == "on")
    {
      Serial.println("LED ON");
      digitalWrite(LED_BUILTIN, HIGH);
    }
    else if (serial_command == "off")
    {
      Serial.println("LED OFF");
      digitalWrite(LED_BUILTIN, LOW);
    }
    else
    {
      Serial.print("Unknown command: "+serial_command);
    }

    
//    switch(serial_command)
//    {
//      case "on":
//        Serial.println("LED ON");
//        digitalWrite(LED_BUILTIN, HIGH);
//      case "off":
//        Serial.println("LED OFF");
//        digitalWrite(LED_BUILTIN, LOW);
//    }
  }

}

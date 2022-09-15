
// Digital ports for LED connections
int LED1 = 2;
int LED2 = 3;
bool LED_status = false;

int i = 0;
int wait_time = 5e3; // Time to wait in 

int const command_len = 2;
char command[command_len];  // Character array to be sent on serial communication

void setup() 
{
  Serial.begin(9600);
  Serial.println("Setup begin");
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);

  // Start with LEDs off
  digitalWrite(LED1, LED_status);
  digitalWrite(LED2, LED_status);
}

void loop()
{
  //wait_msec(wait_time);
  //Serial.println("i = " + String(i));

  if (Serial.available()>0)
  {
    decode_serial();
    LED_command(int(command[0]), command[1]);
  }
  // LED_status = !LED_status;
  // digitalWrite(LED1, LED_status);
  // digitalWrite(LED2, !LED_status);
}

void wait_msec(int interval)
{
  unsigned long previous_time = millis(); // Start timer (i.e. 0)
  unsigned long current_time = millis();  // Update timer (i.e., 0+t)
  while((current_time-previous_time) < interval)
  {
    current_time = millis();
  }
}

void decode_serial()
{
  wait_msec(100); // Wait 100 msec for complete serial transmission
  
  for (int i=0; i<command_len; i++)
  {
    command[i] = Serial.read();
    Serial.println("i: " + String(i));
    Serial.println("c: " + String(command[i]));
  }
}

void LED_command(int LED, char status)
{
  bool LED_status;

  if (status == 'i')
  {
    LED_status = true;
  }
  else if (status == 'o')
  {
    LED_status = false;
  }
  else
  {
    Serial.println("Wrong command");
  }

  digitalWrite(LED, LED_status);
}

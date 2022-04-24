//collects data from an analog sensor (Distance)

//int A0 = 0;              // analog pin used to connect the sharp sensor
int val = 0;                    // variable to store the values from sensor(initially zero)

void setup()
{
  Serial.begin(9600);           
}
 
void loop()
{
  val = analogRead(A0);  // reads the value of the sharp sensor
  Serial.println(val);          // prints the value of the sensor to the serial monitor
  delay(400);                   // wait for this much time before printing next value
}

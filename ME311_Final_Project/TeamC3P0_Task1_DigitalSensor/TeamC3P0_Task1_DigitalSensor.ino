//This is the code for digital IR Sensor (1 or 0)

#include <Romi32U4.h>
Romi32U4LCD lcd;


void setup(){
Serial.begin(9600);
}
void loop(){
  
  Serial.print("Digital Signal:");
  Serial.println(digitalRead(A0),BIN);    //Print to Serial Monitor
  
  delay(50);
  lcd.gotoXY(0, 0);
  lcd.print(digitalRead(A0));             //Print digital signal to LCD
  
}

#pragma once


void print_scan(){
  lcd.autoscroll();
  for(int i = 0; i < 5; i++)
  {
      lcd.gotoXY(i*4, 0);
      lcd.print(radar[i]);
  } 
}

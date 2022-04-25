#pragma once

#include <Servo.h>

Servo radar_servo;  // create servo object to control a servo

int * radar;
int servo_pin = 0;

int * sweep() {
  // Sweeps accross and reads the values.
  static int radar[5];
  int initial_position = 30;
  byte radar_pin = A2;
  
  // Get the current radar readings.
  for (int position = 0; position <= 5; position += 1) {
    radar_servo.write(initial_position + position * 40);
    delay(1000);
    radar[position] = analogRead(radar_pin);
    delay(1000);
  } // end of radar readings
  return radar;
}

#include <Romi32U4.h>
#include <Wire.h>
#include <Servo.h>
#include "sweep.h"
#include "farthest.h"
#include "controls.h"
//#include "functions.h"
#include "gyro.h"

int distance;
int state = 0;
byte radar_pin = A2;
int choice = 3;
int straight_count = 0;
int heading_count = 0;

void setup() {
  Serial.begin(9600);
  turnSensorSetup();
  radar_servo.attach(servo_pin);
  forward();
}

void loop() {
  
    
    display_heading();
    //Serial.println(straight_count);
    // Read the sensor
    if (analogRead(radar_pin) > 600){
      // Stop the motors; 
      motors.setSpeeds(0,0);
      // Get the current radar readings.
      radar = sweep();
      //print_scan();
      // Which of the 5 radar reading is the farthest.
      choice = farthest();
      // Turn the robot that direction 
        if (choice < 3){
         left();
         straight_count = 0;
        } else if(choice > 3) {
         right();
         straight_count = 0;
        }
   
      // Check if we have gone straight 3 time. If so
      // in reverse because we are stuck probabaly.
      if(straight_count != 2){
        forward();
        straight_count++;
      } else {
        reverse();
        straight_count = 0;
      }
    }
} // end of loop

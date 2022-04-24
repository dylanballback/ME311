// This demo shows how the Romi can use its gyroscope to detect
// when it is being rotated, and use the motors to resist that
// rotation.  If you place the robot on a rotating platter and
// spin it, it should keep pointing in approximately the same
// direction.
//
// This code was tested on a Romi with a 120:1 Mini Plastic
// Gearmotor HP and 6 NiMH batteries.  If you have different
// motors or batteries, you might need to adjust the PID
// constants.
//
// Be careful to not move the robot for a few seconds after
// starting it while the gyro is being calibrated.  During the
// gyro calibration, the yellow LED is on and the words "Gyro
// cal" are displayed on the LCD.
//
// After the gyro calibration is done, press button A to start
// the demo.  If you try to turn the Romi, or put it on a surface
// that is turning, it will drive its motors to counteract the
// turning.
//
// This sketch only uses the Z axis of the gyro, so it is
// possible to pick up the Romi, rotate it about its X and Y
// axes, and then put it down facing in a new position.
//
// To run this sketch, you will need to install the LSM6 library:
//
// https://github.com/pololu/lsm6-arduino


#include <Wire.h>
#include <Romi32U4.h>
#include <LSM6.h>
#include "TurnSensor.h"

int main_trigger0 = 0;
int main_trigger1 = 0;

int trigger = 0;
const int16_t maxSpeed = 200;
// vairable to save the last executed time for code block 1
unsigned long lastExecutedMillis_1 = 0;
unsigned long lastExecutedMillis_2 = 0;
unsigned long currentMillisss;

// first interval is after 10 seconds 
#define EXE_INTERVAL_1 19000
// second interval is after 1.5 seconds
#define EXE_INTERVAL_2 1500


Romi32U4LCD lcd;
Romi32U4ButtonA buttonA;
Romi32U4Motors motors;
LSM6 imu;

void setup()
{
  Serial.begin(9600);
  turnSensorSetup(); //handles the calibration of gyro and displaying current heading
  setIncline_speed(0);
  trigger = 0;
  main_trigger0 = 0;
  main_trigger1 = 0;
  delay(5000); //delay 5 seconds
  
}

void loop()
{
  // Read the gyro to update turnAngle, the estimation of how far
  // the robot has turned, and turnRate, the estimation of how
  // fast it is turning.
  currentMillisss = millis();
  
  //Serial.println(trigger);

  
  if (main_trigger0 == 0){
    //this will run after 10 seconds
    if (currentMillisss - lastExecutedMillis_1 >= EXE_INTERVAL_1) {
      lastExecutedMillis_1 = currentMillisss; // save the last executed time
      //setIncline_speed(100);
      //lcd.clear();
      //lcd.gotoXY(0,0);
      //lcd.print(getConverted_x());
      //Serial.print(getIncline_speed());
      //int motorspeed = getIncline_speed();
      //motors.setSpeeds(motorspeed);
      //Serial.print("     ");
      Serial.println("I waited 10 seconds");
      trigger = 1; 
      Serial.println(trigger); 
      main_trigger0 = 1;
    }
  }

  if (main_trigger1 == 0){
    if (currentMillisss - lastExecutedMillis_2 >= EXE_INTERVAL_1 + EXE_INTERVAL_2) {
      lastExecutedMillis_2 = currentMillisss; // save the last executed time
      Serial.println("I have waited an additional 2 seconds");
      trigger = 2;
      Serial.println(trigger); 
      main_trigger1 = 1;     
      }
  }
}
/*    //Turn Sensor Code
  turnSensorUpdate();
  
  if (trigger == 0){
      Serial.println("Case 0");
      //Run the rotation return code here
      // Calculate the motor turn speed using proportional and
      // derivative PID terms.  Here we are a using a proportional
      // constant of 15 and a derivative constant of 1/73.
      int32_t turnSpeed = -(int32_t)turnAngle / (turnAngle1 / 15)
        - turnRate / 73;
      
      // Constrain our motor speeds to be between
      // -maxSpeed and maxSpeed.
      turnSpeed = constrain(turnSpeed, -maxSpeed, maxSpeed);
      //Serial.println(turnSpeed);
      motors.setSpeeds(-turnSpeed, turnSpeed);
      if ((getheading() < 3 ) || (getheading() > -3))
      {
        lcd.gotoXY(0, 0);
        lcd.print(F("Heading"));
        lcd.gotoXY(0, 1);
        int heading = getheading();
        lcd.print(heading);
        lcd.print("      ");
      }
  }
  else if (trigger == 1){
      Serial.println("Case 1");
      //int32_t leftmtoor = 110;
      //int32_t rightmtoor = 100;
      //Run the go foward for 2 seconds code here 
      //motors.setSpeeds(leftmtoor, rightmtoor);
      motors.setSpeeds(110, 100);
  }
  else if (trigger == 2){
      Serial.println("Case 2");
      //Run the go foward for 2 seconds code here
      //int32_t leftmtoor = 0;
      //int32_t rightmtoor = 0;
      //Run the go foward for 2 seconds code here 
      //motors.setSpeeds(leftmtoor, rightmtoor);
      if (getIncline_speed() == 0){
        motors.setSpeeds(110, 100);
      }
      if (getIncline_speed() == 1){
        motors.setSpeeds(0, 0);
      }   
  }
}

  */
  
  /*
  
  //Serial.println(getheading());
  Serial.println(getConverted_x());
  

  if ((getheading() > 5) || (getheading() > -5))
  {
    // Calculate the motor turn speed using proportional and
    // derivative PID terms.  Here we are a using a proportional
    // constant of 15 and a derivative constant of 1/73.
    int32_t turnSpeed = -(int32_t)turnAngle / (turnAngle1 / 15)
      - turnRate / 73;
    
    // Constrain our motor speeds to be between
    // -maxSpeed and maxSpeed.
    turnSpeed = constrain(turnSpeed, -maxSpeed, maxSpeed);
    //Serial.println(turnSpeed);
    motors.setSpeeds(-turnSpeed, turnSpeed);
    if ((getheading() < 3 ) || (getheading() > -3))
    {
      lcd.gotoXY(0, 0);
      lcd.print(F("Heading"));
      lcd.gotoXY(0, 1);
      int heading = getheading();
      lcd.print(heading);
    }
  else if ((getConverted_x() > 10 ) || (getConverted_x() < -10))
  {
    motors.setSpeeds(100, 100);
  }
  }
  */

  
    

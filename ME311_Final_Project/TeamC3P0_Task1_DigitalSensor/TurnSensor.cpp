// Turnsensor.h and TurnSensor.cpp provide functions for
// configuring the romi's LSM6DS33 gyro, calibrating it, and
// using it to measure how much the robot has turned about its Z
// axis.

#include <Wire.h>
#include "TurnSensor.h"

int32_t myvar;

float converted_x;
float getConverted_x() {return converted_x;}

int32_t incline_speed;
int32_t getIncline_speed() {return incline_speed;}
void setIncline_speed(int32_t x) {incline_speed=x;}

int32_t getheading() {return myvar;}
// turnAngle is a 32-bit unsigned integer representing the amount
// the robot has turned since the last time turnSensorReset was
// called.  This is computed solely using the Z axis of the gyro,
// so it could be inaccurate if the robot is rotated about the X
// or Y axes.
//
// Our convention is that a value of 0x20000000 represents a 45
// degree counter-clockwise rotation.  This means that a uint32_t
// can represent any angle between 0 degrees and 360 degrees.  If
// you cast it to a signed 32-bit integer by writing
// (int32_t)turnAngle, that integer can represent any angle
// between -180 degrees and 180 degrees.
uint32_t turnAngle = 0;
uint32_t turnxAngle = 0;

// turnRate is the current angular rate of the gyro, in units of
// 0.035 degrees per second.
int16_t turnRate;
float deg_sec; //turn rate in degrees per second

// This is the average reading obtained from the gyro's Z axis
// during calibration.
int16_t gyroOffsetz;
int16_t gyroOffsetx;


// This variable helps us keep track of how much time has passed
// between readings of the gyro.
uint16_t gyroLastUpdate = 0;


//My vars for filter
uint32_t current_angle = 0;

uint32_t original_heading = 0;

#define EXE_INTERVAL_2 5
//unsigned long lastExecutedMillis_2 = 2;
unsigned long currentMillis;
unsigned long turnOnDelay =2000;  //added
unsigned long buttonPushedMillis; //added
//Change for filter
int timeConst = 2;
float alpha = 0;
float timediff = 0;
float unfiler_data_x = 0;
float filtered = 0;



//int incline_speed;




// This should be called in setup() to enable and calibrate the
// gyro.  It uses the LCD, yellow LED, and button A.  While the LCD
// is displaying "Gyro cal", you should be careful to hold the robot
// still.



void turnSensorSetup(){
  Wire.begin();
  imu.init();

  imu.enableDefault();

  // Set the gyro full scale to 1000 dps because the default
  // value is too low, and leave the other settings the same.
  imu.writeReg(LSM6::CTRL2_G, 0b10001000);

  // Turn on the yellow LED in case the LCD is not available.
  ledYellow(1);

  // Delay to give the user time to remove their finger.
  delay(500);

  // Calibrate the gyro.
  int32_t totalz = 0;
  int32_t totalx = 0;
  for (uint16_t i = 0; i < 1024; i++)
  {
    // Wait for new data to be available, then read it.
    while(!imu.readReg(LSM6::STATUS_REG) & 0x08);
    imu.read();

    // Add the Z axis reading to the total.
    totalz += imu.g.z;
    totalx += imu.a.x;

  }
  ledYellow(0);
  gyroOffsetz = totalz / 1024;
  gyroOffsetx = totalx / 1024;

  
  lcd.clear();

  // Display the angle (in degrees from -180 to 180) until the
  // user presses A.
  turnSensorReset();
  while (!buttonA.getSingleDebouncedRelease()){
        turnSensorUpdate();
        myvar = ((((int32_t)turnAngle >> 16) * 360) >> 16);
        
        //Serial.println(myvar);
        //int32_t myvar_filtered = filter_data(myvar); //this is for the filter
        lcd.gotoXY(0,0);
        lcd.print(myvar);
        lcd.print(F("       "));
        //Serial.println(myvar_filtered);

        Serial.print("Digital Signal:");        //Digital IR sensor
        Serial.println(digitalRead(A0),BIN);    //Print to Serial Monitor
        lcd.gotoXY(0, 1);                       //Display on Second Line of code
        lcd.print(digitalRead(A0));             //Print digital signal to LCD
  

      //If button B is pressed it will recalculate the gyro bias
      if (buttonB.isPressed()){
        totalz = 0; //reset last gyro offset total
        totalx = 0; //reset last gyro offset total
        lcd.clear();
        lcd.print(F("Gyro cal"));
        // Turn on the yellow LED in case the LCD is not available.
        ledYellow(1);
        // Delay to give the user time to remove their finger.
        delay(500);
        // Calibrate the gyro.
        // Calibrate the gyro.
        int32_t totalz = 0;
        int32_t totalx = 0;
        for (uint16_t i = 0; i < 1024; i++)
        {
          // Wait for new data to be available, then read it.
          while(!imu.readReg(LSM6::STATUS_REG) & 0x08);
          imu.read();
      
          // Add the Z axis reading to the total.
          totalz += imu.g.z;
          totalx += imu.a.x;
      
        }
        ledYellow(0);
        gyroOffsetz = totalz / 1024;
        gyroOffsetx = totalx / 1024;
        turnSensorReset();
        lcd.clear();
        
        }
  }
}

void display_heading(){
    int32_t current_heading = ((((int32_t)turnAngle >> 16) * 360) >> 16);
    lcd.clear();
    lcd.print(F("Heading"));
    lcd.gotoXY(0, 1);
    //lcd.print(current_heading);
  }

// This should be called to set the starting point for measuring
// a turn.  After calling this, turnAngle will be 0.
void turnSensorReset(){
  gyroLastUpdate = micros();
  turnAngle = 0;
}

// Read the gyro and update the angle.  This should be called as
// frequently as possible while using the gyro to do turns.
void turnSensorUpdate(){
  // Read the measurements from the gyro.
  imu.readGyro();

  int16_t x = imu.a.x - gyroOffsetx;
  int16_t y = imu.a.y;
  int32_t magnitudeSquared = (int32_t)x * x + (int32_t)y * y;
  
  turnRate = imu.g.z - gyroOffsetz;
  myvar = ((((int32_t)turnAngle >> 16) * 360) >> 16);
  float x_rate = imu.g.x - gyroOffsetx;
  float x_angle = (57.2958*asin(0.000061*x_rate)); // x angle in degrees
  converted_x = (x_angle*12.69);

  
  
   
  // Figure out how much time has passed since the last update (dt)
  uint16_t m = micros();
  uint16_t dt = m - gyroLastUpdate;
  gyroLastUpdate = m;

  // Multiply dt by turnRate in order to get an estimation of how
  // much the robot has turned since the last update.
  // (angular change = angular velocity * time)
  int32_t d = (int32_t)turnRate * dt;
  float dxx = (int32_t)x_rate *dt;

  // The units of d are gyro digits times microseconds.  We need
  // to convert those to the units of turnAngle, where 2^29 units
  // represents 45 degrees.  The conversion from gyro digits to
  // degrees per second (dps) is determined by the sensitivity of
  // the gyro: 0.035 degrees per second per digit.
  //
  // (0.035 dps/digit) * (1/1000000 s/us) * (2^29/45 unit/degree)
  // = 7340032/17578125 unit/(digit*us)
  turnAngle += (int64_t)d * 7340032 / 17578125;
  turnxAngle += (int64_t)dxx * 7340032 / 17578125;
  
  deg_sec = turnAngle * 0.035; // Degree per Second
  //Serial.print("   ");
  //Serial.print(deg_sec);
  //Serial.println("  deg sec");
  

  int16_t forwardSpeedright;
  int16_t forwardSpeedleft;
  int16_t turnSpeed;
  if (converted_x > 50)
  {
    //Serial.println("INCLINE MORE THAN 8 DEGREES");
    // We are on an incline of more than 5 degrees, so
    // try to face uphill using a feedback algorithm.
    //turnSpeed = y / 2;
    setIncline_speed(1);
  
    forwardSpeedleft = 120;
    forwardSpeedright = 100;
    ledYellow(1);
  }
  else
  {
    // We not on a noticeable incline, so don't turn.
    
    //Serial.println("FLAT SURFACE");
    setIncline_speed(0);
    turnSpeed = 0;
    forwardSpeedleft = 0;
    forwardSpeedright = 0;
    ledYellow(0);
  }

  
}

/*
int32_t filter_data(int32_t unfiltered_data){
  currentMillis = millis();
  if (currentMillis - lastExecutedMillis_2 >= EXE_INTERVAL_2) {
    timediff = currentMillis - lastExecutedMillis_2;
    lastExecutedMillis_2 = currentMillis; // save the last executed time
    alpha = timediff / (timeConst + timediff);
    int32_t filtered = filtered + alpha * (unfiltered_data - filtered);
    Serial.print("Unfiltered: ");
    Serial.print(unfiltered_data);
    Serial.print(",");
    Serial.print("Filtered: ");
    Serial.println(filtered);
    //Serial.print("\n");
  }

}
*/

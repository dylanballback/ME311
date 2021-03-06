// Turnsensor.h and TurnSensor.cpp provide functions for
// configuring the Romi's gyro, calibrating it, and using it to
// measure how much the robot has turned about its Z axis.

#pragma once

#include <Romi32U4.h>
#include <LSM6.h>


// This constant represents a turn of 45 degrees.
const int32_t turnAngle45 = 0x20000000;

// This constant represents a turn of 90 degrees.
const int32_t turnAngle90 = turnAngle45 * 2;

// This constant represents a turn of approximately 1 degree.
const int32_t turnAngle1 = (turnAngle45 + 22) / 45;

const int32_t current_heading = 0;

float getConverted_x();
int32_t getIncline_speed();
void setIncline_speed(int32_t x);


// These are defined in TurnSensor.cpp:
void turnSensorSetup();
void turnSensorReset();
void turnSensorUpdate();
int32_t getheading();
//int32_t filter_data(int32_t unfiltered_data);
extern uint32_t turnAngle;
extern int16_t turnRate;

// These objects must be defined in your sketch.
extern Romi32U4ButtonA buttonA;
extern Romi32U4ButtonB buttonB;
extern Romi32U4LCD lcd;
extern LSM6 imu;

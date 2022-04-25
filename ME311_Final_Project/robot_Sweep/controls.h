#pragma once

Romi32U4Motors motors;

void right(){
  // Counter rotate move right.
   motors.flipRightMotor(true);
   motors.setLeftSpeed(50);
   motors.setRightSpeed(50);
   delay(500);
   motors.setSpeeds(0,0);
}

void left(){
  // Counter rotate move left.
   motors.flipLeftMotor(true);
   motors.setLeftSpeed(50);
   motors.setRightSpeed(50);
   delay(500);
   motors.setSpeeds(0,0);
}

void forward(){
  // Move both motors forward

    // Set both motors back to default direction. 
    motors.flipLeftMotor(false);
    motors.flipRightMotor(false);
    // Center the ASRM
    radar_servo.write(110);
    delay(1000);
    // Drive forward
    motors.setSpeeds(50,50);
}

void reverse(){
   // Set both motors back to default direction. 
    motors.flipLeftMotor(true);
    motors.flipRightMotor(true);
    // Center the ASRM
    radar_servo.write(110);
    // Drive forward
    motors.setSpeeds(50,50);    
    delay(2000);
    forward();
}

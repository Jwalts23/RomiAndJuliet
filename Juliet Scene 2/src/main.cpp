
// This example reads the raw values from the LSM6DS33
// accelerometer and gyro and prints those raw values to the
// serial monitor.

#include <Romi32U4.h>
#include "chassis.h"
#include "Timer.h"

Romi32U4ButtonA buttonA;
Romi32U4ButtonB buttonB;
Romi32U4ButtonC buttonC;


PIDController leftMotorController(5,1,0,500); //start with  Kp = 1
PIDController rightMotorController(5,1,0,500);

Timer waitTimer(1000);

Chassis chassis;

  void setup()
  {
     chassis.Init();
pinMode(7,INPUT);
pinMode(12,OUTPUT);
 }

float IReffort=0.0;



void loop()
 {

  if(buttonB.getSingleDebouncedPress())
  {
    chassis.cal = true;
  }
  if (chassis.cal)
  {
    chassis.calculateXOffset();
  }
  if(buttonC.getSingleDebouncedPress())
  {
    chassis.state=chassis.driveFlat;
    //scene=DriveUpRamp;
  }

  if(PIDController::readyToPID)
  {
    PIDController::readyToPID=0;
    //JulietScene();
    chassis.driveUpRamp();
    chassis.UpdateSpeeds();
  }

 }





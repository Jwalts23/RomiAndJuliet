// This example reads the raw values from the LSM6DS33
// accelerometer and gyro and prints those raw values to the
// serial monitor.

#include <Romi32U4.h>
#include "chassis.h"

Romi32U4ButtonA buttonA;
Romi32U4ButtonB buttonB;
Romi32U4ButtonC buttonC;
//Romi32U4Motors motor;
Chassis chassis;

  void setup()
  {
     chassis.Init();

 }

// bool showAcc = true;
// bool showGyro = false;

void loop()
{
  if(buttonB.isPressed())
  {
    chassis.cal = true;
  }
  
  if (chassis.cal){
    chassis.calculateXOffset();
  }
  if(buttonC.isPressed())
  {
    delay(2000);
   
    chassis.state= chassis.driveFlat;
    
  }
//Serial.println();

 if(PIDController::readyToPID)
    {
      PIDController::readyToPID=0;
    chassis.driveUpRamp();
    chassis.UpdatePitch();
    }
}

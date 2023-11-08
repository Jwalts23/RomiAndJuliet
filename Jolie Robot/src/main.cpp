
// This example reads the raw values from the LSM6DS33
// accelerometer and gyro and prints those raw values to the
// serial monitor.
// Juliet Scene 2 ramp scene
#include <Romi32U4.h>
#include "chassis.h"
#include "Timer.h"

Romi32U4ButtonA buttonA;
Romi32U4ButtonB buttonB;
Romi32U4ButtonC buttonC;
Romi32U4Motors motor;

PIDController leftMotorController(5, 1, 0, 500); // start with  Kp = 1
PIDController rightMotorController(5, 1, 0, 500);
int LEDPin = 4;
int LimitSwitch = 12;
int IREmitter = 11;
Timer IRTimer(250);
Timer BlinkTimer(140);
Timer waitTimer(1000);
Chassis chassis;

void setup()
{
  chassis.Init();
  motor.init();
  pinMode(LEDPin, INPUT);
  pinMode(LimitSwitch, OUTPUT);
  pinMode(IREmitter, OUTPUT);
}


float IReffort = 0.0;
void IRblink()
{
  if (chassis.state == chassis.StopAtTop || chassis.state == chassis.START_Blinking)
  {
    if (IRTimer.isExpired())
    {
      if (OCR1C == 210)
      {
        OCR1C = 0;
      }
      else
      {
        OCR1C = 210;
      }
    }
  }
  else
  {
    OCR1C = 0;
  }
}

void LEDBlink()
{
  if (chassis.state == chassis.StopAtTop || chassis.state == chassis.START_Blinking)
  {
    if (BlinkTimer.isExpired())
    {
      digitalWrite(12, digitalRead(12) == 1 ? 0 : 1);
    }
  }
  else
  {
    digitalWrite(12, 0);
  }
}

void loop()
{
  if (buttonB.getSingleDebouncedPress())
  {
    chassis.cal = true;
  }
  if (chassis.cal)
  {
    chassis.calculateXOffset();
  }
  if (buttonC.getSingleDebouncedPress())
  {
    chassis.state = chassis.driveFlat;
  }

  IRblink();
  LEDBlink();
  if (PIDController::readyToPID)
  {
    PIDController::readyToPID = 0;
    chassis.driveUpRamp();
    chassis.UpdateSpeeds();
  }
}

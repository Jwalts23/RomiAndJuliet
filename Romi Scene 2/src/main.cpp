/*
 * Code for using TCC4 for precision PID timing.
 * You'll need to set TOP to set the interval
 *
 * This code adds the ability to tune the gains and change the targets
 */

#include <Romi32U4.h>
#include "chassis.h"
#include "Timer.h"
#include "IRDirectionFinder.h"

Romi32U4ButtonC buttonC;
Chassis chassis;
Romi32U4Encoders encoders;
IRDirectionFinder irFinder;
Timer BlinkTimer(140);
Timer IRTimer2(250);
int IREmitter = 11;
int IRReceiver = 18;
Romi32U4Motors irmotor;
Timer IRCameraRead(45);
Timer IRTimerReceiver(50);
bool found = false;

int i = 0;
int g = 0;
int z = 0;
float irEffort = 0.0;

enum STATE
{
  start,
  blink,
  wait,
  follow,
  stop
};
STATE state = start;

Timer waitTimer(1000);

void setup()
{
  Serial.begin(115200);
  irFinder.begin();
  chassis.Init();
  pinMode(12, OUTPUT);
  pinMode(4, INPUT);
  pinMode(IREmitter, OUTPUT);
  pinMode(IRReceiver, INPUT);
}

float centerIR()
{
  float error = 0.0;
  float effort = 0.0;
  double Kp = 0.07;
  irFinder.requestPosition();
  if (irFinder.available())
  {
    Point point = irFinder.ReadPoint(0);
    if (point.x != 1023)
    {
      error = (point.x) - 511.0;
      effort = Kp * error;
    }
  }
  else
  {
    Serial.println("Device not available!");
  }
  return effort;
}
void LEDBlink()
{
  if (state == blink || state == wait || state == follow)
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
bool FindIR()
{
  irFinder.requestPosition();
  if (irFinder.available())
  {
    Point point = irFinder.ReadPoint(0);
    if (point.x != 1023)
    {
      found = true;
    }
  }
  else
  {
    Serial.println("Device not available!");
  }
  return found;
}

void loop()
{
  LEDBlink();
  if (PIDController::readyToPID) // timer flag set
  {
    PIDController::readyToPID = 0;
    switch (state)
    {
    case start:
      if (IRTimerReceiver.isExpired() && digitalRead(IRReceiver) == 0)
      {
        Serial.println("State changing to blink");
        state = blink; // blink so we know he got the signal
      }
      break;

    case blink:
      if (IRTimerReceiver.isExpired() && (digitalRead(IRReceiver) == 1))
      {
        state = wait;
      }
      break;
      
    case wait:
      FindIR();
      if (found)
      {
        z++;
        if (z > 25)
        {
          state = follow;
        }
      }
      break;

    case follow:
      irEffort = centerIR();
      chassis.SetTargetSpeeds(7 - irEffort, 7 + irEffort);
      if (digitalRead(4) == 0) // reading limit switch
      {
        state = stop;
      }
      break;

    case stop:
      chassis.Stop();
      break;
    }
    chassis.UpdateSpeeds();
  }
}

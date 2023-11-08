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
int IREmitter=11;
Romi32U4Motors irmotor;
Timer IRCameraRead(45);
bool found=false;

int i =0;
float irEffort=0.0;

Timer waitTimer(1000);

void setup()
{
  Serial.begin(115200);
  irmotor.init();
  irFinder.begin();
  chassis.Init();
  pinMode(12,OUTPUT);
  pinMode(4,INPUT);
  pinMode(IREmitter,OUTPUT);
  
}

enum STATE {DR_IDLE, DR_WAITING, DR_DRIVING,DRIVING_TOWARDS,DRIVETOROMI,LINEUP};
STATE state = DR_IDLE;

enum DESTINATION {DEST_NONE, DEST_A, DEST_B, DEST_C,DEST_MOVE_TO};
DESTINATION destination = DEST_NONE;

void HandleButton(DESTINATION d)
{
 // Serial.println(d);
  destination = d;
  waitTimer.reset(1000);

  state = DR_WAITING;
}
void IRblink()
{
     if(state==LINEUP)
    {
     if(IRTimer2.isExpired())
    {
        if(OCR1C==210)
        {
            OCR1C=0;
        }
        else{ OCR1C=210;}
    }
    }
   else
   {
     OCR1C=0;
   }
  }

float centerIR()
{

  float error=0.0;
  float effort=0.0;
  double Kp=0.08;
  irFinder.requestPosition();
  if (irFinder.available()) {

    Point point = irFinder.ReadPoint(0);
   
       if(point.x!=1023)
       {
        error=(point.x)-530.0;
        effort=Kp*error;    
       }
      }  
     
  else{
    Serial.println("Device not available!");
     }
   return effort;
}

float centerIR2()
{

  float error2=0.0;
  float effort2=0.0;
  double Kp2=0.1;
  irFinder.requestPosition();
  if (irFinder.available()) {

    Point point = irFinder.ReadPoint(0);
   
       if(point.x!=1023)
       {
        error2=(point.x)-485.0;
        effort2=Kp2*error2;    
       }
      }  
     
  else{
    Serial.println("Device not available!");
     }
   return effort2;
}
void LEDBlink()
{
  if(state==DR_IDLE)
  {
    digitalWrite(12,0);
  }
  else if(BlinkTimer.isExpired())
  {
    if(digitalRead(12)==1)
    {
      digitalWrite(12,0);
    }
    else
    {
      digitalWrite(12,1);
    }
  }
}
bool FindIR()
{
  irFinder.requestPosition();
  if (irFinder.available()) {

    Point point = irFinder.ReadPoint(0);
   
       if(point.x!=1023)
       {
        found=true;
       }
      }  
     
  else{
    //Serial.println("Device not available!");
     }
   return found;
}

void afterCircle()
{
  chassis.SetTargetSpeeds(-2,2);
  chassis.UpdateSpeeds();
  FindIR();
  if (found)
  {     
         chassis.SetTargetSpeeds(0,0);
         chassis.Stop();
         destination=DEST_NONE;
         state=DRIVING_TOWARDS; 
  }
}

void afterCircle2()
{
  chassis.SetTargetSpeeds(-4,4);
  chassis.UpdateSpeeds();
  FindIR();
  if (found)
  {     
         chassis.SetTargetSpeeds(0,0);
         chassis.Stop();
         destination=DEST_NONE;
         state=LINEUP; 
  }
}
void HandleTimerExpired(void)
{
    switch(state)
    {
      case DR_WAITING:
        if(destination == DEST_A) //drive straight
        {
          chassis.SetTargetPosition(30,30);
          state = DR_DRIVING;
        }
        else if(destination == DEST_B) //spin in place
        {
         
         chassis.SetTargetPosition(60,0);
          state = DR_DRIVING;
        }
        else if(destination == DEST_C) //curl
        {
          chassis.SetTargetPosition(30,-30);
          state = DR_DRIVING;
        }
        break;
    }  
}



void loop() 
{
 
  if(buttonC.getSingleDebouncedPress()) HandleButton(DEST_C); 
  LEDBlink();
  IRblink();

  if(PIDController::readyToPID) //timer flag set
  {
    PIDController::readyToPID = 0;
    chassis.UpdatePose();
    switch (state)
    {
    case DR_DRIVING:
      if(chassis.MoveToPoint())
        {
          chassis.SetTargetSpeeds(0,0);
          if (i<2)
          {
          destination=destination==DEST_A?DEST_C:DEST_A;
          state = DR_WAITING;
          i++;
          }
          else
          {
            afterCircle();
          }
        }
      break;

        case DRIVING_TOWARDS:
        irEffort=centerIR();
        chassis.SetTargetSpeeds(15-irEffort,15+irEffort);
   
        if(digitalRead(4)==0)
        {
           chassis.Stop();
           chassis.SetTargetSpeeds(0,0);
           state=DRIVETOROMI;
           destination=DEST_NONE;
           found=false;
        }
        break;

       case DRIVETOROMI:
       afterCircle2();
       break;
       
       case LINEUP:
          irEffort=centerIR2();
          chassis.SetTargetSpeeds(10-irEffort,10+irEffort);
          //when limit switch is hit
         if(digitalRead(4)==0)
        {
          chassis.SetTargetSpeeds(0,0);
          chassis.Stop();
          state=DR_IDLE;
          destination=DEST_NONE;
        }
        break;

    }
          chassis.UpdateSpeeds();
    }
if(waitTimer.isExpired()) HandleTimerExpired();

  }


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

//const uint8_t IR_PIN = A6;

Romi32U4ButtonA buttonA;
Romi32U4ButtonB buttonB;
Romi32U4ButtonC buttonC;
Chassis chassis;
Romi32U4Encoders encoders;
IRDirectionFinder irFinder;
Romi32U4Motors irmotor;

int i =0;
int j = 0;

Timer waitTimer(1000);
Timer BlinkTimer(140);

const int IR_Pin=18;
const int IR_emitter=20;
Timer IRTimer(50);
Timer IRTimer2(250);
enum STATE {DR_IDLE, DR_WAITING, DR_DRIVING,LEDBLINKSTOP};
STATE State = DR_IDLE;

enum DESTINATION {DEST_NONE, DEST_A, DEST_B, DEST_C,DEST_D,DEST_MOVE_TO};
DESTINATION destination = DEST_NONE;

void setup()
{
  Serial.begin(115200);
  //while(!Serial) {}  //IF YOU DON'T COMMENT THIS OUT, YOU MUST OPEN THE SERIAL MONITOR TO START
  //irmotor.init();
  irFinder.begin();
  chassis.Init();
  pinMode(IR_emitter,OUTPUT);
  pinMode(12,OUTPUT);
  pinMode(4,INPUT);
    
}
void LEDBlink()
{
  if(State==LEDBLINKSTOP)
  {
    digitalWrite(12,0);
  }
 else if(BlinkTimer.isExpired())
  {
    if(digitalRead(12)==1)
    {
      digitalWrite(12,0);
    }
    else{
      digitalWrite(12,1);
    }
  }
}
void IRblink()
{
  if(digitalRead(4)==0)
    {
      destination=DEST_NONE;
      State=LEDBLINKSTOP;
      return;
    }

  if(State==LEDBLINKSTOP)
    {
      digitalWrite(IR_emitter,0);
      //OCR1C=0;// stops Emitter Blinking
    }
    else 
    {
        digitalWrite(IR_emitter,1);
      }
    
  }

void HandleButton(DESTINATION d)
{
  //Serial.println(d);
  destination = d;
  waitTimer.reset(1000);
  State = DR_WAITING;
}

void HandleTimerExpired(void)
{
    switch(State)
    {
      case DR_WAITING:
        if(destination == DEST_A) //drive straight
        {
          chassis.SetTargetPosition(30,30);
          State = DR_DRIVING;
        }
        else if(destination == DEST_B) //spin in place
        {
         chassis.SetTargetPosition(30,-20);
          State = DR_DRIVING;
        }
        else if(destination == DEST_C) //curl
        {
          chassis.SetTargetPosition(30,-30);
          State = DR_DRIVING;
        }
        else if(destination == DEST_D) //curl
        {
          chassis.SetTargetPosition(30,-45);
          State = DR_DRIVING;
        }
        break;
        case LEDBLINKSTOP:
        if(j<4){
          chassis.SetTargetSpeeds(-10,-10);
          j++;
        }
        else if (j>=4){
          chassis.Stop();
        }
        break;
    }  
}

void loop() 
{
  if(buttonA.getSingleDebouncedPress()) HandleButton(DEST_A);
  if(buttonB.getSingleDebouncedPress()) HandleButton(DEST_B);
  if(buttonC.getSingleDebouncedPress()) HandleButton(DEST_C);

  IRblink();
  LEDBlink();
 
  if(PIDController::readyToPID) //timer flag set
  {
    PIDController::readyToPID = 0;
  
    // Serial.print(State);
    // Serial.print('\t');

    chassis.UpdatePose();
    if(State == DR_DRIVING)
    {
     if(chassis.MoveToPoint())
        {
         // Serial.println("reached!");
          chassis.SetTargetSpeeds(0,0);
          if (i<2)
          {
          destination=(destination==DEST_A)?DEST_C:DEST_A;
          State = DR_WAITING;
          i++;
          }
          else 
          {
          destination=DEST_B;
          State=DR_WAITING;
          }
        }   
    }
      
     chassis.UpdateSpeeds();
  }
if(waitTimer.isExpired())HandleTimerExpired();
}


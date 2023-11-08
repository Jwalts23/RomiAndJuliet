#include <Arduino.h>
#include "Timer.h"
#include "Romi32U4Motors.h"
#include "chassis.h"

Timer IRTimer2(250);
Timer BlinkTimer(140);
int IRemitter=20;
int IRReceiver=18;
Romi32U4Motors irmotor;
Chassis chassis;


void IRon()
{
  digitalWrite(IRemitter,1);
}

void setup() {
  pinMode(IRemitter,OUTPUT);
  pinMode(IRReceiver,INPUT);
  pinMode(12,OUTPUT);
  pinMode(4,INPUT);

}
void LEDBlink()
{

 if(BlinkTimer.isExpired())
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

void loop() 
{
  IRon();
  LEDBlink();
  Serial.println(digitalRead(IRReceiver));
  
    if(digitalRead(4)==0) //reading limit switch
  {
    chassis.move(0);
  }

 else if(digitalRead(IRReceiver)==0)
  {
    chassis.move(50); //move Straight
  }

}
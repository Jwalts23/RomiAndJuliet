

#include <Arduino.h>
#include <Wire.h>
#include "chassis.h"
#include "Romi32U4Motors.h"
#include <IRDirectionFinder.h>
#include "Timer.h"

IRDirectionFinder irFinder;
Chassis chassis;
Romi32U4Motors irmotor;
Timer IRCameraRead(100);


 float center()
{
  float error=0.0;
  float effort=0.0;
  double Kp=0.2;
  irFinder.requestPosition();
  if (irFinder.available()) {

    Point point = irFinder.ReadPoint(0);
   
       if(point.x!=1023)
       {
        error=(point.x)-511.0;
        effort=Kp*error;    
        Serial.print(point.x);
        Serial.print("\t");
        Serial.print(effort);
        Serial.print("\n");  
       }
      }  
     
  else{
    Serial.println("Device not available!");
   }
   return effort;
}

void setup()
{
  Serial.begin(115200);
  irmotor.init();
  irFinder.begin();
  chassis.Init();
}

void loop()
{
//center();
if(IRCameraRead.isExpired())
{
Serial.println(digitalRead(18));
}
}

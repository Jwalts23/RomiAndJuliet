#include "chassis.h"

#include <math.h>

volatile int16_t countsLeft = 0;
volatile int16_t countsRight = 0;

Romi32U4Encoders encoder;
Romi32U4Motors motors;

Chassis::Chassis(void) 
    //we'll use the member initializer capabilities of C++ to set up the controllers
    : leftMotorController(5, 1, 0, 500), rightMotorController(5, 1, 0, 500)  
    {
        SetTargetSpeeds(0, 0);
    }

void Chassis::Init(void)
{  
    noInterrupts(); //disable interupts while we mess with the Timer4 registers
  
    //sets up timer 4
    TCCR4A = 0x00; //disable some functionality -- no need to worry about this
    TCCR4B = 0x0B; //sets the prescaler -- look in the handout for values
    TCCR4C = 0x04; //toggles pin 6 at the timer frequency
    TCCR4D = 0x00; //normal mode

    /*
    * EDIT THE LINE BELOW WITH YOUR VALUE FOR TOP
    */

    OCR4C = 249;   //TOP goes in OCR4C 
    timestepMS = 16; //should correspond to your choice for TOP

    TIMSK4 = 0x04; //enable overflow interrupt

    interrupts(); //re-enable interrupts

    //pinMode(6, OUTPUT); //COMMENT THIS OUT TO SHUT UP THE PIEZO!!!
}

void Chassis::UpdatePose(void)
{
    //TO BE COMPLETED BY THE STUDENT

    //conversion from ticks/interval to cm/sec
    float conversionToCMPerSec = (2 *PI)*(wheel_diam/2)*(1000.0/(ticks_per_rotation*16.0)) ; //YOU'LL NEED TO CALCULATE THIS VALUE
    
    float spLeft = speedLeft * conversionToCMPerSec;
    float spRight = speedRight * conversionToCMPerSec;

    //average speed
    float u_0 = (spLeft+spRight)/2.0; //YOU'LL NEED TO ADD THIS EXPRESSION

    //omega
    float omega = (spRight-spLeft)/wheel_track; //YOU'LL NEED TO ADD THIS EXPRESSION

    //simple first-order method -- not sufficient for class
    float dt = timestepMS / 1000.0; //SET timestepMS IN THE CONSTRUCTOR

    float R=(wheel_track/2)*((spLeft+spRight)/(spRight-spLeft));
    //YOU'LL NEED TO CALCULATE THESE
    if (omega!=0){

     x +=R*(sin( theta + (omega*dt)) - sin(theta)) ;
     y +=R*(cos(theta)-cos(theta+(omega*dt)));
     theta += (omega*dt);
    }
    else{
        x += u_0* cos(theta)*dt ;
        y += u_0 * sin(theta)*dt;
        theta += (omega*dt);
   }

    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.print(theta); 
    Serial.print('\t');
     Serial.print(error_distance);
     Serial.print("\n");
}

void Chassis::UpdateSpeeds(void)
{
    /*
     * Do PID stuffs here. Note that we turn off interrupts while we read countsLeft/Right
     * so that it won't get accidentally updated (in the ISR) while we're reading it.
     */
    noInterrupts();

    speedLeft = countsLeft - prevEncLeft;
    prevEncLeft = countsLeft;
    speedRight = countsRight - prevEncRight;
    prevEncRight = countsRight;

    interrupts();

    int16_t errorLeft = targetSpeedLeft - speedLeft; //calculate the error
    float effortLeft = leftMotorController.ComputeEffort(errorLeft); //calculate effort from error
    
    motors.setLeftEffort(effortLeft); 

    int16_t errorRight = targetSpeedRight - speedRight; //calculate the error
    float effortRight = rightMotorController.ComputeEffort(errorRight); //calculate effort from error
    
    motors.setRightEffort(effortRight); 
}

 boolean Chassis::MoveToPoint(){

     double KP_distance=0.36;
     double KP_angle=5.0;//0.9
     
     double deltaY=y_target-y;
     double deltaX=x_target-x;

     th_target=atan2(deltaY,deltaX);

     error_distance=sqrt((pow((deltaX),2.0) + (pow((deltaY),2.0))));
     error_theta=th_target-theta;
     
     if(error_theta > PI)
     {
         error_theta -=2*PI;
     }
     if(error_theta < - PI)
     {
         error_theta+=2*PI;
     }

     targetSpeedLeft=KP_distance*error_distance - KP_angle*error_theta;
     targetSpeedRight=KP_distance*error_distance + KP_angle*error_theta;


    
  if( error_distance <= 5.0)
     {
        return true;
     }
     return false;
 }

 
/*
 * ISR for timing. On overflow, it takes a 'snapshot' of the encoder counts and raises a flag to let
 * the main program it is time to execute the PID calculations.
 */
ISR(TIMER4_OVF_vect)
{
  //Capture a "snapshot" of the encoder counts for later processing
  countsLeft = encoder.getCountsLeft();
  countsRight = encoder.getCountsRight();

  PIDController::readyToPID = 1;
}

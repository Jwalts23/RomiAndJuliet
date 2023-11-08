#include "chassis.h"
#include <math.h>
#include "Timer.h"
#include <math.h>


Romi32U4Encoders encoder;
Romi32U4Motors motors;
volatile int16_t countsLeft = 0;
volatile int16_t countsRight = 0;
Timer WallTimer(50);
LSM6 imu;

int k = 0;



Chassis::Chassis(void) 
    //we'll use the member initializer capabilities of C++ to set up the controllers
    : leftMotorController(5, 1, 0, 500), rightMotorController(5, 1, 0, 500)  
    {
        SetTargetSpeeds(0, 0);
    }

void Chassis::Init(void)
{  
   IMUsetup();
   IRsetup();
    noInterrupts(); //disable interupts while we mess with the Timer4 registers
  
    //sets up timer 4
    TCCR4A = 0x00; //disable some functionality -- no need to worry about this
    TCCR4B = 0x0B; //sets the prescaler -- look in the handout for values
    TCCR4C = 0x04; //toggles pin 6 at the timer frequency
    TCCR4D = 0x00; //normal mode

    OCR4C = 249;   //TOP goes in OCR4C 
    timestepMS = 16; //should correspond to your choice for TOP

    TIMSK4 = 0x04; //enable overflow interrupt

    interrupts(); //re-enable interrupts

    //pinMode(6, OUTPUT); //COMMENT THIS OUT TO SHUT UP THE PIEZO!!!
}
void Chassis::Stop(void){
  SetTargetSpeeds(0,0);
  motors.setEfforts(0,0);
  
}
void Chassis::UpdatePose(void)
{
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



 double Chassis::calculateXOffset()
 { 
  
   if(imu.getStatus() & 0x01)
  {
     sum+=imu.a.x;
     count++;
  }

   if(count==200)
   {
     accXoffset=sum/200.0;
   sum=0.0;
   count=0;
   cal = false;
   }

   return accXoffset;
 }

 double Chassis::UpdatePitch(){

  if(imu.getStatus() & 0x01)
  {
  
    imu.read();

   gyroReading= imu.dps.y * PI / 180;// - gyroBias; //(in rad/s)

   estimatedPitchAngle=correctedPitchAngle+(1.0/52.0)*(gyroReading); 
   pitchAngle=atan2(-(imu.a.x - accXoffset),imu.a.z);

    correctedPitchAngle=(K*estimatedPitchAngle)+((1.0-K)*pitchAngle);

    gyroBias += epsilon*(estimatedPitchAngle-pitchAngle)/(1.0/52.0);

  }
    return correctedPitchAngle;
 }

void Chassis::WallFollowIR(float targetDistanceIR)
{
        irEffort=IRloop(targetDistanceIR);
        SetTargetSpeeds(10-irEffort,10+irEffort);
        UpdateSpeeds();
}

float previousPitch=0.0;

 void Chassis::driveUpRamp(void){
   switch(state){

    case start:

    break;

    case driveFlat:

      if (UpdatePitch()*(180.0/PI)<5 && UpdatePitch()*(180.0/PI)> -5)
      {
        WallFollowIR(7.5);
      }
      else
      {
        state = onRamp;
      }
     break;
     
    case onRamp:
      if (UpdatePitch()*(180.0/PI)>4 || UpdatePitch()*(180.0/PI)<-4)
      {
        irEffort=IRloop(7.5);
        SetTargetSpeeds(10-irEffort,10+irEffort);
      }
      else
      {
      state=StopAtTop;
      }
    break;

    case StopAtTop:
      // stop the motors
      SetTargetSpeeds(0, 0);
      motors.setEfforts(0,0);
      // current angle
      previousPitch=UpdatePitch()*(180/PI);
      //inialize delay variable
      a++;
      // this delay negates the the initial transition from the ground to the ramp
      if (a>200)
        {
          state=DownTheRamp;
        }
      break;

    case DownTheRamp:
    // compare the previous angle to the current angle
    // stop when the previous angle is less than the current angle
    if(previousPitch <= UpdatePitch()*(180/PI))
      {
        SetTargetSpeeds(-10,-10);
      }
    else
      {
        state=DriveAfterTipping;
      }
    break;
        
    case DriveAfterTipping:
    // drive straight for a little bit after ramp
    {
      // if the angle is around zero which means that the Juliet is back on the ground
      // and the delay k is greater than or equal to 100 then change state
      if (UpdatePitch()*(180.0/PI)<5 && UpdatePitch()*(180.0/PI)> -5 && k>=100)
      {
        state=NextPositionForScene;
      }
      else 
      {
        k++;
        SetTargetSpeeds(-10,-10);
        Serial.println(k);
      }
    }
          break;
           
    case NextPositionForScene:
    // drive straight using dead reckoning
    if (l<80)
    {
      SetTargetSpeeds(-10,-10);
      l++;
    }
    else
    {
      l++;
      Stop();
      // delay before moving to the next state
      if (l>110)
      {
        l=0;
        state= turn;
      }
    }
    break;

    case turn:
    // dead reckon 90 degrees
      if (h<150)
      {
        SetTargetSpeeds(-5,5);
        h++;
      }
      else
      {   
        h++;
        Stop();
        // delay before moving onto next state
        if (h>=180)
        {
          h=0;
          state=driveStraight;
        }
      }
    break;

    case driveStraight:
    // drive straight using dead reckoning
    if (l<180)
    {
      SetTargetSpeeds(-10,-10);
      l++;
    }
    else
    {
      l++;
      Stop();
      if (l>200)
      {
        l=0;
        state= turn2;
      }
    }
    break;

    case turn2:
    // dead reckon 90 degrees
    if (h<150)
    {
      SetTargetSpeeds(-5,5);
      h++;
    }
    else
    {
      h++;
      Stop();
      // delay before moving to next state
      if(h>=175)
      {
        h=0;
        state=STOP;
      }
    }
    break;

    case STOP:
    // stop
      SetTargetSpeeds(0, 0);
      motors.setEfforts(0,0);
      done = true;
    break;
   }
  
 }

 //IR sensor
double kpIR=0.0;
double kdIR=0.0;
double errorIR=0.0;
double effortIR=0.0;
float targetLeft =0.0;
float targetRight=0.0;
float curEffortIR=0.0;
float prevEffortIR=0.0;
uint16_t IRPin=22;

void Chassis::IRsetup()
{
pinMode(IRPin, INPUT);
}

float Chassis::IRloop(float targetDistanceIR)
{
  kpIR=1.0;
  kdIR=0.0;
float distance;

float ADC_result_raw =0.0;
float ADC_result=0.0;
ADC_result_raw=analogRead(IRPin);
ADC_result=(ADC_result_raw/1024.0)*5.0;
distance= pow((14.1/ADC_result),(1.0/0.812));
errorIR=distance - targetDistanceIR;

 if(errorIR > 0|| errorIR < 0)
  { 
   effortIR=kpIR*errorIR + kdIR*(curEffortIR-prevEffortIR);
  }
  else
  {
   effortIR=0.0;
  }

prevEffortIR=curEffortIR;

return effortIR;
}
 void Chassis::IMUsetup()
{    

  Wire.begin();
  if (!imu.init())
  {
    // Failed to detect the LSM6.
    ledRed(1);
    while(1)
    {
      Serial.println(F("Failed to detect the LSM6."));
      delay(100);
    }
  }
   imu.enableDefault();

  // Set the gyro full scale and data rate
  imu.setGyroDataOutputRate(LSM6::ODR26);

  // Set the accelerometer full scale and data rate
  imu.setAccDataOutputRate(LSM6::ODR26);
 
    estimatedPitchAngle=0.0;
    pitchAngle=0.0;
    gyroReading=0.0;
    prevEstimatedAngle=0.0;
    correctedPitchAngle=0.0;

  }

void Chassis::PID(){
     // reset the flag
    PIDController::readyToPID = 0;
    // for tracking previous counts

    float speedLeft=0.0;
    float speedRight=0.0;

    /*
     * Do PID stuffs here. Note that we turn off interupts while we read countsLeft/Right
     * so that it won't get accidentally updated (in the ISR) while we're reading it.
     */
    noInterrupts();

    speedLeft = countsLeft - prevLeft;
    prevLeft = countsLeft;
    speedRight = countsRight - prevRight;
    prevRight = countsRight;

    interrupts();

    int16_t errorLeft = targetLeft - speedLeft; //calculate the error
    float effortLeft = leftMotorController.ComputeEffort(errorLeft); //calculate effort from error
    
    motors.setLeftEffort(effortLeft); 

    int16_t errorRight = targetRight - speedRight; //calculate the error
    float effortRight = rightMotorController.ComputeEffort(errorRight); //calculate effort from error
    
    motors.setRightEffort(effortRight);

    interrupts();
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

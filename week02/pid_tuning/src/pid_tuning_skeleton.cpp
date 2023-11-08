/*
 * Code for using TCC4 for precision PID timing.
 * You'll need to set TOP to set the interval
 * 
 * This code adds the ability to tune the gains and change the targets
 */

#include <Romi32U4.h>
#include "serial_comm.h"
#include "PIDcontroller.h"
#include "Timer.h"
using namespace std;


PIDController leftMotorController(5,1,0,500); //start with  Kp = 1
PIDController rightMotorController(5,1,0,500);
volatile uint8_t PIDController::readyToPID = 0; //a flag that is set when the PID timer overflows

Romi32U4Motors motors;
Romi32U4Encoders encoders;
 Timer wallTimer(50);

volatile int16_t countsLeft = 0;
volatile int16_t countsRight = 0;
float targetLeft =0.0;
float targetRight=0.0;
double effort_Ultra=0.0;

uint16_t IRPin=22;
uint32_t INTERVAL=39;
float ADC_result_raw =0.0;
float ADC_result=0.0;
uint32_t lastTime;
/*
 * Code for interfacing a 32U4 with the SR-HR04 ultrasonic sensor. 
 * 
 * This uses the Input Capture feature of the ATmega32U4 (e.g., Leonardo) to get precision readings.
 * Specifically, you must connect the pulse width pin to pin 13 (ICP3) on the 32U4.
 * You are welcome to use whatever pin you want for triggering a ping, just be sure to change it from the default.
 * 
 * The input capture first looks for a rising edge, then a falling edge
 * The difference between the two is the pulse width, which is a direct measurement 
 * of the (round trip) timer counts to hear the echo.
 * 
 * But note that the timing is in timer counts, which must be converted to time.
 */
volatile uint16_t pulseStart = 0;
volatile uint16_t pulseEnd = 0;

//Ultrasonic Sensor
double kp=0.0;
double kd=0.0;
double error=0.0;
double effort=0.0;
float curEffort=0.0;
float prevEffort=0.0;

//IR sensor
double kpIR=0.0;
double kdIR=0.0;
double errorIR=0.0;
double effortIR=0.0;
float curEffortIR=0.0;
float prevEffortIR=0.0;


//define the states for the echo capture
enum PULSE_STATE {PLS_IDLE, PLS_WAITING_LOW, PLS_WAITING_HIGH, PLS_CAPTURED};

//and initialize to IDLE
volatile PULSE_STATE pulseState = PLS_IDLE;

//this may be most any pin, connect the pin to Trig on the sensor
const uint8_t trigPin = 14;

//for scheduling pings
uint32_t lastPing = 0;
const uint32_t PING_INTERVAL = 100; //ms

/*
 * Commands the ultrasonic to take a reading
 */
void CommandPing(int trigPin)
{
  cli(); //disable interrupts

  TIFR3 = 0x20; //clear any interrupt flag that might be there

  TIMSK3 |= 0x20; //enable the input capture interrupt
  TCCR3B |= 0xC0; //set to capture the rising edge on pin 13; enable noise cancel

  sei(); //re-enable interrupts

  //update the state and command a ping
  pulseState = PLS_WAITING_LOW;
  
  digitalWrite(trigPin, HIGH); //command a ping by bringing TRIG HIGH
  delayMicroseconds(10);      //we'll allow a delay here for convenience; it's only 10 us
  digitalWrite(trigPin, LOW);  //must bring the TRIG pin back LOW to get it to send a ping
}

void ultrasetup()
{
   Serial.begin(115200);
  //while(!Serial) {} //you must open the Serial Monitor to get past this step!
  Serial.println("setup");

  noInterrupts(); //disable interupts while we mess with the control registers
  
  //sets timer 3 to normal mode (16-bit, fast counter)
  TCCR3A = 0; 
  
  interrupts(); //re-enable interrupts

  //note that the Arduino machinery has already set the prescaler elsewhere
  //so we'll print out the value of the register to figure out what it is
  Serial.print("TCCR3B = ");
  Serial.println(TCCR3B, HEX);

  pinMode(trigPin, OUTPUT);
  pinMode(13, INPUT); //explicitly make 13 an input, since it defaults to OUTPUT in Arduino World (LED)

  lastPing = millis();

}

void ultraloop(double targetDistance) 
{
  //schedule pings roughly every PING_INTERVAL milliseconds
  uint32_t currTime = millis();
  if((currTime - lastPing) >= PING_INTERVAL && pulseState == PLS_IDLE)
  {
    lastPing = currTime;
    CommandPing(trigPin); //command a ping
  }
  if(pulseState == PLS_CAPTURED) //we got an echo
  {
    //update the state to IDLE
    pulseState = PLS_IDLE;

    /*
     * Calculate the length of the pulse (in timer counts!). Note that we turn off
     * interrupts for a VERY short period so that there is no risk of the ISR changing
     * pulseEnd or pulseStart. The way the state machine works, this wouldn't 
     * really be a problem, but best practice is to ensure that no side effects can occur.
     */
    noInterrupts();
    uint16_t pulseLengthTimerCounts = pulseEnd - pulseStart;
    interrupts();
    
    //EDIT THIS LINE: convert pulseLengthTimerCounts, which is in timer counts, to time, in us
    //You'll need the clock frequency and the pre-scaler to convert timer counts to time
    
    uint32_t pulseLengthUS = pulseLengthTimerCounts*4; //pulse length in us

    //EDIT THIS LINE AFTER YOU CALIBRATE THE SENSOR: put your formula in for converting us -> cm
    float distancePulse =(pulseLengthUS-43.599)/51.623;
    //distance in cm

  kp=2.0;
  kd=1.0;
  error= distancePulse-targetDistance;

  if(error > 0|| error < 0)
  { 
    curEffort= kp*error + kd*(curEffort-prevEffort);
    
  }
  else
  {
   curEffort=0.0;
  }
  effort=curEffort;
 prevEffort=curEffort;

}

}

void IRsetup()
{
pinMode(IRPin, INPUT);
}



void IRloop(float targetDistanceIR)
{
  kpIR=1.0;
  kdIR=0.0;
    
//uint32_t currentTime = millis();
float distance;

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
   effortIR;
  }

//effortI=curEffortIR;
prevEffort=curEffortIR;

Serial.print(ADC_result_raw);
Serial.print('\t');
Serial.print(ADC_result);
Serial.print('\t');
Serial.print(distance);
Serial.print('\n');

}



// Serial.print(ADC_result_raw);
// Serial.print('\t');
// Serial.print(ADC_result);
// Serial.print('\t');
// Serial.print(distance);
// Serial.print('\n');



void setup()
{
  
  Serial.begin(115200);
  //while(!Serial) {}  //IF YOU DON'T COMMENT THIS OUT, YOU MUST OPEN THE SERIAL MONITOR TO START
  Serial.println("Hi");

  noInterrupts(); //disable interupts while we mess with the Timer4 registers
  
  //sets up timer 4
  TCCR4A = 0x00; //disable some functionality -- no need to worry about this
  TCCR4B = 0x0B; //sets the prescaler -- look in the handout for values
  TCCR4C = 0x04; //toggles pin 6 at the timer frequency
  TCCR4D = 0x00; //normal mode

  /*
   * EDIT THE LINE BELOW WITH YOUR VALUE FOR TOP
   */

  OCR4C = 223;   //TOP goes in OCR4C 

  TIMSK4 = 0x04; //enable overflow interrupt
  
  interrupts(); //re-enable interrupts
  ultrasetup();
  IRsetup();
 // pinMode(6, OUTPUT); //COMMENT THIS OUT TO SHUT UP THE PIEZO!!!
}

static int16_t prevLeft = 0;
static int16_t  prevRight = 0;

void loop() 
{    
  
//ultraloop(40);

if(wallTimer.isExpired())
{
  IRloop(7);
  targetLeft=15-effortIR;
  targetRight=15+effortIR;
}


  if(PIDController::readyToPID) //timer flag set
  {
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

  /* for reading in gain settings
   * CheckSerialInput() returns true when it gets a complete string, which is
   * denoted by a newline character ('\n'). Be sure to set your Serial Monitor to 
   * append a newline
   */
  if(CheckSerialInput()) {ParseSerialInput();}
}

/*
 * ISR for timing. On overflow, it takes a 'snapshot' of the encoder counts and raises a flag to let
 * the main program it is time to execute the PID calculations.
 */
ISR(TIMER4_OVF_vect)
{
  //Capture a "snapshot" of the encoder counts for later processing
  countsLeft = encoders.getCountsLeft();
  countsRight = encoders.getCountsRight();
  PIDController::readyToPID = 1;
}


/*
 * ISR for input capture on pin 13. We can precisely capture the value of TIMER3
 * by setting TCCR3B to capture either a rising or falling edge. This ISR
 * then reads the captured value (stored in ICR3) and copies it to the appropriate
 * variable.
 */
ISR(TIMER3_CAPT_vect)
{
  if(pulseState == PLS_WAITING_LOW) //we're waiting for a rising edge
  {
    pulseStart = ICR3; //copy the input capture register (timer count)
    TCCR3B &= 0xBF;    //now set to capture falling edge on pin 13
    pulseState = PLS_WAITING_HIGH;
  }

  else if(pulseState == PLS_WAITING_HIGH) //waiting for the falling edge
  {
    pulseEnd = ICR3;
    pulseState = PLS_CAPTURED; //raise a flag to indicate that we have data
  }
}
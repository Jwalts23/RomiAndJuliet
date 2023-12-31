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

#include <Arduino.h>
#include "Romi32U4Encoders.h"
#include "Romi32U4Motors.h"

using namespace std;

Romi32U4Motors motors;

volatile uint16_t pulseStart = 0;
volatile uint16_t pulseEnd = 0;

int i;
int k;
double sum=0.0;
double average=0.0;
double median = 0.0;
double d[5];
double c[5];
double e[5];


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

void setup()
{
   Serial.begin(115200);
   i=0;
   k=0;
   sum=0.0;
   average=0.0;

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

  Serial.println("/setup");
  Serial.println("Roopsa_Ultrasonic_20cm");
}

double rollingAverage(float DistancePulse)
{
 
  sum=0.0;
  average=0.0;

 if(k < 5)
 { 
   d[k]=DistancePulse;
   k++;   
 }

 else{
   
   sum= d[0]+d[1]+d[2]+d[3]+d[4];
   average= sum/5.0;
   d[0]=d[1];
   d[1]=d[2];
   d[2]=d[3];
   d[3]=d[4];
   d[4]=DistancePulse;
 }
 return average;
}
 
 void sortArray(double arr[])
 {

double temp;
  for(int i=0;i< 5;i++)
  {
    for(int j=i+1;j<4;j++)
    {
      if(arr[i] >arr[j] )
      {
        temp=arr[i];
        arr[i]=arr[j];
        arr[j]=temp;
      }
    }
  }
 }

 double rollingMedian(float distance)
 {
   median=0.0;
   if(i < 5)
 { 
   c[i]=distance;
   e[i]=distance;
   i++;
       
 }
 else if(i==5)
 {
   sortArray(e);
   median=e[2];
   i++;
 }

 else{
   // pushing the new value into the array
   c[0]=c[1];
   c[1]=c[2];
   c[2]=c[3];
   c[3]=c[4];
   c[4]=distance;

  e[0]=c[0];
  e[1]=c[1];
  e[2]=c[2];
  e[3]=c[3];
  e[4]=c[4];
   sortArray(e);
   median=e[2];
   
 }

return median;

 }

double curEffort;
double prevEffort;
double Effort;
double kp;
double kd;
float error;
void standoff(float targetDistance,float current_Distance)
{
  kp=4;
  error= current_Distance-targetDistance;
  if(error > 0|| error < 0)
  {
    
    Effort= kp*error  + kd*(curEffort-prevEffort);
    
    motors.setEfforts(curEffort,curEffort);
    prevEffort= curEffort;
  }
  else
  {
    motors.setEfforts(0,0);
  }
}

void loop() 
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

  //Printing the values

  Serial.print(error);
  Serial.print('\t');
  //Serial.print(effort);
  //Serial.print('\t');
     Serial.print(rollingMedian(distancePulse));
     Serial.print('\t');
    Serial.print(millis());
    Serial.print('\t');
   Serial.print(pulseLengthTimerCounts);
    Serial.print('\t');
   Serial.print(pulseLengthUS);
    Serial.print('\t');
     Serial.print(distancePulse);
     Serial.print('\t');
  Serial.print(rollingAverage(distancePulse));
      Serial.print('\t');
     Serial.print('\n');

standoff(20.0,rollingAverage(distancePulse));

}}



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

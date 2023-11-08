#include <PIDcontroller.h>
#include <math.h>

/*
 * ComputeEffort takes the current error as an argument. You could just as easily pass the current
 * speed and target, but I've chosen to pass the error.
 */

volatile uint8_t PIDController::readyToPID = 0; //a flag that is set when the PID timer overflows

float PIDController::ComputeEffort(float error)
{
    // Kp and Ki only -- add KD if you want to

    //BE SURE TO SET THE K AND BOUND WHEN YOU CREATE THE OBJECT
    currError = error; //store the current error
    sumError += error;

    if(errorBound > 0)
    {
        if(sumError > errorBound) sumError = errorBound;
        if(sumError < -errorBound) sumError = -errorBound;
    }

    currEffort = Kp * currError + Ki * sumError;

    return currEffort;
}

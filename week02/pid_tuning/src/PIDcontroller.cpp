#include <PIDcontroller.h>
#include <math.h>

/*
 * ComputeEffort takes the current error as an argument. You could just as easily pass the current
 * speed and target, but I've chosen to pass the error.
 */

float PIDController::ComputeEffort(float error)
{
    //THIS ONLY DOES PROPORTIONAL CONTROL FOR NOW!
    //YOU'LL NEED TO ADD INTEGRAL AND DERIVATIVE CONTROL!
    //The necessary variables have been created for you -- see the header file

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

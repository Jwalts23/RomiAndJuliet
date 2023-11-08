#pragma once

#include <Arduino.h>

class PIDController
{
private:
    //float target = 0;

    float Kp, Ki, Kd;
    float currError = 0;
    float prevError = 0;
    
    float sumError = 0;
    float errorBound = 0; //defaults to zero -- you might want to add a cap!

    float currEffort = 0;
public:
    static volatile uint8_t readyToPID;   //a flag that is set when the PID timer overflows

public:
    /*
     * The constructor takes a kp term as a minimum. You can pass other gains and a cap on the integral
     * if you want, but they default to 0, if you don't.
     */ 
    PIDController(float p, float i = 0, float d = 0, float bound = 0) 
        : Kp(p), Ki(i), Kd(d), errorBound(bound) 
        {}

    float ComputeEffort(float error);
    float SetKp(float k) {return Kp = k;}
    float SetKi(float k) {sumError = 0; return Ki = k;}
    float SetKd(float k) {return Kd = k;}
};

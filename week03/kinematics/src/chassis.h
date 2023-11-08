#ifndef __CHASSIS_H
#define __CHASSIS_H

#include <math.h> 
#include <Romi32U4.h>
#include "PIDcontroller.h"

class Chassis
{
private:
    //parameters -- these will need to be updated after you do your experiments
    float wheel_track = 14.5; //cm
    float wheel_diam = 6.9; //cm
    float ticks_per_rotation = 1442.0; // from the datasheet

    //current Pose
    float x = 0;
    float y = 0;
    float theta = 0;

    //current target
    float x_target = 0;
    float y_target = 0;
    float th_target = 0; //may or may not be specified

    //target wheels speeds
    float targetSpeedLeft = 0;
    float targetSpeedRight = 0;

    //for calculating speeds
    int16_t prevEncLeft = 0;
    int16_t prevEncRight = 0;

    //wheel speed controllers
    PIDController leftMotorController; 
    PIDController rightMotorController; 

    //actual speed
    int16_t speedLeft = 0;
    int16_t speedRight = 0;

    //be sure to set this -- it needs to match your "readyToPID" period
    uint32_t timestepMS = 16; // in ms


    float error_distance=0.0;
    float error_theta=0.0;

public:
    Chassis(void);
    void Init(void);
    void SetTargetSpeeds(float left, float right) { targetSpeedLeft = left; targetSpeedRight = right; }
    void SetTargetPosition(float xt, float yt) { x_target = xt; y_target = yt; }
    void UpdatePose(void);
    void UpdateSpeeds(void);
    boolean MoveToPoint(void);
};

#endif
#include "PID.h"

float PD_Controller::next(float desired, float actual, float desiredDot, float actualDot)
{
    float error = desired - actual;
    float errorDot = desiredDot - actualDot;

    float effort = Kp * error + Kd * errorDot;
    if(effort > MaxEffort)
    {
        effort = MaxEffort;
    }

    if(effort < -MaxEffort)
    {
        effort = -MaxEffort;
    }

    return effort;
}

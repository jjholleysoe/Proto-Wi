#include "PID.h"

float PD_Controller::next(float desired, float actual, float desiredDot, float actualDot, float wheelVelDesired, float wheelVelActual)
{
    float error = desired - actual;
    float errorDot = desiredDot - actualDot;
    float wheelVelError = wheelVelDesired - wheelVelActual;

    float effort = Kp * error + Kd * errorDot + Kw * wheelVelError;
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

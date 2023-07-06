/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
aaaaaaaaaaaaaa
Module: PID.cpp
Author: Salar Shekari
Date started: Unknown

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/


#include "../inc/PID.h"
#include <iostream>

namespace dynamics
{

PID::PID(double _dt)
{
    dt  = _dt;
//     min = _min;
//     max = _max;
}


double PID::PID_Output(double _SetPoint, double _current_state, double _last_state, double min_sat, double max_sat, double Kp, double Ki, double Kd)
{
    SetPoint = _SetPoint;

    current_state = _current_state;
    last_state = _last_state;

    pError = (current_state - SetPoint);
    iError = pError;
    dError = (current_state-last_state)/dt;     // Kick Derivative

    OutputSignal = (pError * Kp) + (dError * Kd) + (iError * Ki);


    if(OutputSignal > max_sat)
    {
        return max_sat;
    }
    else if(OutputSignal < min_sat)
    {
        return min_sat;
    }
    else
    {
        return OutputSignal;
        // return 555.787;
    }

}




}
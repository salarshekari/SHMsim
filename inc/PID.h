/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Header: PID.h
Author: Salar Shekari
Date started: Unknown

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/


#ifndef _PID_H
#define _PID_H

namespace dynamics
{

class PID
{
    public:
        PID(double _dt);

        // void SetInitState(double _current_state, double _last_state);
        double PID_Output(double _SetPoint, double _current_state, double _last_state, double min_sat, double max_sat, double Kp, double Ki, double Kd);

        // double Kp;
        // double Ki;
        // double Kd;

        


    protected:
        double dt;
        // double min_sat;
        // double max_sat;

        double current_state;
        double last_state;


    private:
        double OutputSignal;
        double SetPoint;

        // This Variables, store the Values of Errors
        double pError;
        double iError;
        double dError;


};

}


#endif
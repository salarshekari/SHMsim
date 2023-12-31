/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Header: EOM.h
Author: Salar Shekari
Date started: Unknown

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#ifndef EOM_H
#define EOM_H

#define rad2deg (180.0/3.141592653589793238)
#define Pi 3.141592653589793238462643383279502884197

#include <memory>
#include <deque>
#include <vector>

#include "FGColumnVector3.h"
// #include "FGLocation.h"

class EOM
{
    public:

    /** The current vehicle state vector structure contains the translational and
    angular position, and the translational and angular velocity. */
    struct VehicleStates
    {
        // The acceleration vector of the vehicle with respect the Local Level (NED) frame,
        // expressed in the body system.
        // units m
        JSBSim::FGColumnVector3 XYZ;

        // The velocity vector of the vehicle with respect to the Local Level (NED) frame,
        // expressed in the Local frame.
        // units m/sec 
        JSBSim::FGColumnVector3 XYZ_dot;

        // The acceleration vector of the vehicle with respect to the Local Level (NED) frame,
        // expressed in the Local frame.
        // units m/(sec^2) 
        JSBSim::FGColumnVector3 XYZ_dot_dot;




        // Orientation of the vehicle (phi, theta, psi) in Local Level (NED) Frame
        // units radian
        JSBSim::FGColumnVector3 phi_theta_psi;

        // The angular velocity vector for the vehicle relative to the Local Level (NED) frame,
        // expressed in the same frame.
        // units rad/sec
        JSBSim::FGColumnVector3 phi_theta_psi_dot;
        
        // The angular acceleration vector for the vehicle relative to the Local Level (NED) frame,
        // expressed in the same frame.
        // units rad/(sec^2) 
        JSBSim::FGColumnVector3 phi_theta_psi_dot_dot;

        // std::deque <FGColumnVector3>
        std::deque <JSBSim::FGColumnVector3> dq_phi_theta_psi_dot_dot;
        std::deque <JSBSim::FGColumnVector3> dq_XYZ_dot_dot;
        std::deque <JSBSim::FGColumnVector3> dq_XYZ_dot;
        std::deque <JSBSim::FGColumnVector3> dq_phi_theta_psi_dot;

        // Orientation of the vehicle (phi, theta, psi) in Local Level (NED) Frame
        // units degree
        JSBSim::FGColumnVector3 phi_theta_psi_deg;

        
    };

    struct VehicleParameters
    {
        double mass;
        double arm_len;
        double Ix;
        double Iy;
        double Iz;
        double Jr;

        double Ct;
        double Cm;
    };

    enum eIntegrateType {eNone = 0, eRectEuler, eTrapezoidal, eAdamsBashforth2,
                       eAdamsBashforth3, eAdamsBashforth4, eBuss1, eBuss2, eLocalLinearization, eAdamsBashforth5};

    EOM(double _mass, double _arm_len, double _Ix, double _Iy, double _Iz, double _Jr, double _Ct, double _Cm, double R_b, double rho);
    ~EOM();

    void pos_init(double phi_0, double theta_0, double psi_0, double X0, double Y0, double Z0);
    void rate_init(double phi_dot_0, double theta_dot_0, double psi_dot_0, double udot_0, double vdot_0, double wdot_0);
    void Run(double dt);
    void CalculateUVWdot(void);
    void CalculatePQRdot(void);
    void UpdatePhi_Theta_Psi(void);
    void setMotorSpeed_rad_per_second(std::vector<double> Om);
    void doTrime(std::vector<double> &Om, double );

    void PitchControl(double pitch_setpoint, double dt, double pitch_current_state, double pitch_last_state, double alt_pid);
    void RollControl(double roll_setpoint, double dt, double roll_current_state, double roll_last_state, double alt_pid);
    double AltControl(double alt_setpoint, double dt, double alt_current_state, double alt_last_state);
    void YawControl(double yaw_setpoint, double dt, double psi_current_state, double psi_last_state, double alt_pid);

    void set_alt_gain(double _Kp_Alt, double _Ki_Alt, double _Kd_Alt)
    {Kp_Alt = _Kp_Alt; Ki_Alt = _Ki_Alt; Kd_Alt = _Kd_Alt;}

    void set_pitch_gain(double _Kp_Theta, double _Ki_Theta, double _Kd_Theta)
    {Kp_Theta = _Kp_Theta; Ki_Theta = _Ki_Theta; Kd_Theta = _Kd_Theta;}

    void set_roll_gain(double _Kp_Phi, double _Ki_Phi, double _Kd_Phi)
    {Kp_Phi = _Kp_Phi; Ki_Phi = _Ki_Phi; Kd_Phi = _Kd_Phi;}

    void set_yaw_gain(double _Kp_Psi, double _Ki_Psi, double _Kd_Psi)
    {Kp_Psi = _Kp_Psi; Ki_Psi = _Ki_Psi; Kd_Psi = _Kd_Psi;}
    

    JSBSim::FGColumnVector3 getXYZ() {return vStates.XYZ;}
    JSBSim::FGColumnVector3 getPhi_Theta_Psi_rad() {return vStates.phi_theta_psi;}
    JSBSim::FGColumnVector3 getPhi_Theta_Psi_deg() {return vStates.phi_theta_psi_deg;}

    double getSimTime() {return sim_time;}

    double getX() {return vStates.XYZ(1);}
    double getY() {return vStates.XYZ(2);}
    double getZ() {return vStates.XYZ(3);}

    double getPhi() {return vStates.phi_theta_psi_deg(1);}
    double getTheta() {return vStates.phi_theta_psi_deg(2);}
    double getPsi() {return vStates.phi_theta_psi_deg(3);}

    double getDownAccel() {return vStates.XYZ_dot_dot(3);}

    



    private:
    VehicleStates vStates;
    VehicleParameters vParams;

    double Trim_RPM;
    double Offset;

    double sim_time;

    double g;
    double DiskArea;
    double BladeRadius;

    double current_i_err;
    double last_i_err;

    eIntegrateType integrator_rotational_rate;
    eIntegrateType integrator_translational_rate;
    eIntegrateType integrator_rotational_position;
    eIntegrateType integrator_translational_position;

    double U1;
    double M_phi;
    double M_theta;
    double M_psi;

    double phi;
    double theta;
    double psi;

    double phi_dot;
    double theta_dot;
    double psi_dot;

    double Omega;

    double Omega1;
    double Omega2;
    double Omega3;
    double Omega4;

    double Kp_Phi;
    double Ki_Phi;
    double Kd_Phi;

    double Kp_Theta;
    double Ki_Theta;
    double Kd_Theta;

    double Kp_Psi;
    double Ki_Psi;
    double Kd_Psi;

    double Kp_Alt;
    double Ki_Alt;
    double Kd_Alt;

    void Integrate( JSBSim::FGColumnVector3& Integrand,
                  JSBSim::FGColumnVector3& Val,
                  std::deque <JSBSim::FGColumnVector3>& ValDot,
                  double dt,
                  eIntegrateType integration_type);

    
};


#endif
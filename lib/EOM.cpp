/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Module: EOM.cpp
Author: Salar Shekari
Date started: Unknown

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/


#include "EOM.h"

#include <iostream>
#include <cmath>

using namespace JSBSim;
using namespace std;


EOM::EOM(double _mass, double _arm_len, double _Ix, double _Iy, double _Iz, double _Jr, double _Ct, double _Cm)
{
    vParams.mass = _mass;
    vParams.arm_len = _arm_len;
    vParams.Ix = _Ix;
    vParams.Iy = _Iy;
    vParams.Iz = _Iz;
    vParams.Jr = _Jr;

    vParams.Ct = _Ct;
    vParams.Cm = _Cm;

    g = 9.81;

    integrator_rotational_rate = eRectEuler;
    integrator_translational_rate = eAdamsBashforth2;
    integrator_rotational_position = eRectEuler;
    integrator_translational_position = eAdamsBashforth3;

    vStates.dq_phi_theta_psi_dot_dot.resize(5, FGColumnVector3(0.0,0.0,0.0));
    vStates.dq_XYZ_dot_dot.resize(5, FGColumnVector3(0.0,0.0,0.0));
    vStates.dq_XYZ_dot.resize(5, FGColumnVector3(0.0,0.0,0.0));
    vStates.dq_phi_theta_psi_dot.resize(5, FGColumnVector3(0.0,0.0,0.0));


    UpdatePhi_Theta_Psi();    
}


EOM::~EOM()
{
    std::cout << "\ndestructor was called";
}

void EOM::doTrime(std::vector<double> &Om, double setPoint)
{
  
    while(1)
    {
      
      while((vStates.XYZ_dot_dot(3) < 0))//2.52841e-06))//3.52841e-06))
      {
        
      Omega1 += 0.001;
      Omega2 += 0.001;
      Omega3 += 0.001;
      Omega4 += 0.001;

      Om[0] = Omega1;
      Om[1] = Omega2;
      Om[2] = Omega3;
      Om[3] = Omega4;

      // setMotorSpeed_rad_per_second(Om);
      // CalculateUVWdot();

      Run(0.01);
      std::cout << "\ndown accel is : " << vStates.XYZ_dot_dot(3) << "\n";
      
      }
      while(vStates.XYZ_dot_dot(3) > 0)//2.52841e-06)
      {
      
        
      Omega1 -= 0.001;
      Omega2 -= 0.001;
      Omega3 -= 0.001;
      Omega4 -= 0.001;

      Om[0] = Omega1;
      Om[1] = Omega2;
      Om[2] = Omega3;
      Om[3] = Omega4;


      setMotorSpeed_rad_per_second(Om);
      // CalculateUVWdot();

      std::cout << "\nKoose nanat\n";

      Run(0.01);
      break;
      
      
      }

      Omega1 += 0.001;
      Omega2 += 0.001;
      Omega3 += 0.001;
      Omega4 += 0.001;

      Om[0] = Omega1;
      Om[1] = Omega2;
      Om[2] = Omega3;
      Om[3] = Omega4;

      setMotorSpeed_rad_per_second(Om);

      Run(0.01);

      break;

      setMotorSpeed_rad_per_second(Om);
      CalculateUVWdot();

      std::cout << "\nOmega1 in loop is : " << Omega1 << "\n";
      // std::cout << "\nHeight is : " << vStates.XYZ(3) << "\n";
    }

    Om[0] = Omega1;
    Om[1] = Omega2;
    Om[2] = Omega3;
    Om[3] = Omega4;

    // std::cout << "Omega1 in loop is : " << Om[0] << "\t";


}

void EOM::pos_init(double phi_0, double theta_0, double psi_0, double X0, double Y0, double Z0)
{
    vStates.phi_theta_psi(1) = phi_0;
    vStates.phi_theta_psi(2) = theta_0;
    vStates.phi_theta_psi(3) = psi_0;


    vStates.XYZ(1) = X0;
    vStates.XYZ(2) = Y0;
    vStates.XYZ(3) = Z0;

    Omega1 = 879.9;
  Omega2 = 879.9;
  Omega3 = 879.9;
  Omega4 = 879.9;
}

void EOM::rate_init(double phi_dot_0, double theta_dot_0, double psi_dot_0, double udot_0, double vdot_0, double wdot_0)
{
    vStates.XYZ_dot_dot(1) = udot_0;
    vStates.XYZ_dot_dot(2) = vdot_0;
    vStates.XYZ_dot_dot(3) = wdot_0;

    vStates.phi_theta_psi_dot(1) = phi_dot_0;
    vStates.phi_theta_psi_dot(2) = theta_dot_0;
    vStates.phi_theta_psi_dot(3) = psi_dot_0;
}


void EOM::Run(double dt)
{
  CalculateUVWdot();

  CalculatePQRdot();

  
  Integrate(vStates.phi_theta_psi_dot,   vStates.phi_theta_psi_dot_dot,   vStates.dq_phi_theta_psi_dot_dot,    dt,   integrator_rotational_rate);

  Integrate(vStates.phi_theta_psi,   vStates.phi_theta_psi_dot,   vStates.dq_phi_theta_psi_dot,   dt,   integrator_rotational_position);

  Integrate(vStates.XYZ_dot,   vStates.XYZ_dot_dot,   vStates.dq_XYZ_dot_dot,   dt,   integrator_translational_rate);

  Integrate(vStates.XYZ,   vStates.XYZ_dot,   vStates.dq_XYZ_dot,   dt,   integrator_translational_position);

  
}


void EOM::CalculateUVWdot(void)
{
    U1 = vParams.Ct*(pow(Omega1, 2) + pow(Omega2, 2) + pow(Omega3, 2) + pow(Omega4, 2));
    vStates.XYZ_dot_dot(1) = (cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi))*(1/vParams.mass)*U1;
    vStates.XYZ_dot_dot(2) = (cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi))*(1/vParams.mass)*U1;
    vStates.XYZ_dot_dot(3) = -g + (cos(phi)*cos(theta))*(1/vParams.mass)*U1;
}

void EOM::CalculatePQRdot(void)
{
    UpdatePhi_Theta_Psi();

    M_phi = vParams.Ct*vParams.arm_len*(pow(Omega1,2) - pow(Omega2,2) - pow(Omega3,2) + pow(Omega4,2));
    M_theta = vParams.Ct*vParams.arm_len*(pow(Omega1,2) + pow(Omega2,2) - pow(Omega3,2) - pow(Omega4,2));
    M_psi = vParams.Cm*(-pow(Omega1,2) + pow(Omega2,2) - pow(Omega3,2) + pow(Omega4,2));

    Omega = Omega1 + Omega3 - Omega2 - Omega4;

    vStates.phi_theta_psi_dot_dot(1) = ((vParams.Iy - vParams.Iz)/vParams.Ix)*theta_dot*psi_dot + (vParams.Jr/vParams.Ix)*theta_dot*Omega + (1/vParams.Ix)*M_phi;
    vStates.phi_theta_psi_dot_dot(2) = ((vParams.Iz - vParams.Ix)/vParams.Iy)*phi_dot*psi_dot - (vParams.Jr/vParams.Iy)*phi_dot*Omega + (1/vParams.Iy)*M_theta;
    vStates.phi_theta_psi_dot_dot(3) = ((vParams.Ix - vParams.Iy)/vParams.Iz)*phi_dot*theta_dot + (1/vParams.Iz)*M_psi;
}

void EOM::setMotorSpeed_rad_per_second(std::vector<double> Om)
{
    Omega1 = Om[0];
    Omega2 = Om[1];
    Omega3 = Om[2];
    Omega4 = Om[3];
}




void EOM::UpdatePhi_Theta_Psi(void)
{
    phi = vStates.phi_theta_psi(1);
    theta = vStates.phi_theta_psi(2);
    psi = vStates.phi_theta_psi(3);

    phi_dot = vStates.phi_theta_psi_dot(1);
    theta_dot = vStates.phi_theta_psi_dot(2);
    psi_dot = vStates.phi_theta_psi_dot(3);

    vStates.phi_theta_psi_deg(1) = phi * rad2deg;
    vStates.phi_theta_psi_deg(2) = theta * rad2deg;
    vStates.phi_theta_psi_deg(3) = psi * rad2deg;
}




void EOM::Integrate( FGColumnVector3& Integrand,
                             FGColumnVector3& Val,
                             deque <FGColumnVector3>& ValDot,
                             double dt,
                             eIntegrateType integration_type)
{
  ValDot.push_front(Val);
  ValDot.pop_back();

  switch(integration_type) {
  case eRectEuler:       Integrand += dt*ValDot[0];
    break;
  case eTrapezoidal:     Integrand += 0.5*dt*(ValDot[0] + ValDot[1]);
    break;
  case eAdamsBashforth2: Integrand += dt*(1.5*ValDot[0] - 0.5*ValDot[1]);
    break;
  case eAdamsBashforth3: Integrand += (1/12.0)*dt*(23.0*ValDot[0] - 16.0*ValDot[1] + 5.0*ValDot[2]);
    break;
  case eAdamsBashforth4: Integrand += (1/24.0)*dt*(55.0*ValDot[0] - 59.0*ValDot[1] + 37.0*ValDot[2] - 9.0*ValDot[3]);
    break;
  case eAdamsBashforth5: Integrand += dt*((1901./720.)*ValDot[0] - (1387./360.)*ValDot[1] + (109./30.)*ValDot[2] - (637./360.)*ValDot[3] + (251./720.)*ValDot[4]);
    break;
  case eNone: // do nothing, freeze translational rate
    break;
  case eBuss1:
  case eBuss2:
  case eLocalLinearization:
    throw("Can only use Buss (1 & 2) or local linearization integration methods in for rotational position!");
  default:
    break;
  }
}

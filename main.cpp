#include <iostream>
// #include <vector>
#include "inc/EOM.h"

#define rads2rpm 9.5492968

int main()
{
    double mass = 5.81;         // Weight (Kg)
    double arm_len = 0.18;      // Arm Length (m)
    double Ix = 0.006142;       // X Axis moment of inertia (Kg.m^2) 
    double Iy = 0.006142;       // Y Axis moment of inertia (Kg.m^2)
    double Iz = 0.01338;        // Z Axis moment of inertia (Kg.m^2)
    // double Ct = 0.00000572;     // Thrust Coefficient (N.s^2)   9.62701e-06
    double Ct = 9.62701e-06;     // Thrust Coefficient (N.s^2)   9.62701e-06
    double Cm = 0.000000172;    // Thrust Coefficient (N.s^2)
    double Jr = 0.0001;

    std::vector<double> Om;

    Om.resize(4);

    EOM eomobj(mass, arm_len, Ix, Iy, Iz, Jr, Ct, Cm);

    eomobj.pos_init(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    eomobj.rate_init(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    // Om[0] = 981;//.999999999;
    // Om[1] = 981;//.999999999;
    // Om[2] = 981.00;//.999999999;
    // Om[3] = 981.00;//.999999999;

    
    while(1)
    {
        eomobj.doTrime(Om, 2.1);

        eomobj.setMotorSpeed_rad_per_second(Om);

        eomobj.Run(0.01);

        std::cout << "Omega1 is: " << Om[0] * rads2rpm << "\t\t";
        std::cout << "down accel is : " << eomobj.getDownAccel() << "\t";

        std::cout << "X pos is: " << eomobj.getX() << "\t\t";
        std::cout << "Y pos is: " << eomobj.getY() << "\t\t";
        std::cout << "Z pos is: " << eomobj.getZ() << "\t\t";
        std::cout << "Theta is: " << eomobj.getTheta() << "\t\t";
        std::cout << "Phi is :" << eomobj.getPhi() << "\n";
    }



    return 0;    
}
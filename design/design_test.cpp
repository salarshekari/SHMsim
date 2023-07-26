#include <iostream>
#include <cmath>

#define in2m 0.0254
#define pi 3.1415926535897932
#define rads2rpm 9.5492968
#define rpm2rads 0.1047198

int main()
{
    double W = 4.227;
    double T = W;

    double rho = 1.225;         // Air dencity Kg/m^3
    double R = 15 * in2m;       // Blade Radius in (m)
    double A = pi * R * R;          // Disk area in (m^2)
    
    // double Ct = 0.000118715;            // Thrus coefficient of blade    4.00572e-05  0.000124593
    double Ct = 0.000124593;            // Thrus coefficient of blade

    double Omega_rpm;                   // Motor Angular Velocity in (rpm)
    double Omega_rad;                   // // Motor Angular Velocity in (rad/s)  5074O

    std::cout << "\nEnter the Required Thrust : ";
    std::cin >> T;

    std::cout << "\nEnter rotor angular speed: ";
    std::cin >> Omega_rpm;

    // Omega_rad = 8735 * rpm2rads;   
    Omega_rad = Omega_rpm * rpm2rads;            

    Ct = (2*T) / (rho * A * R * R * Omega_rad * Omega_rad);
    std::cout << "Thrust Ceff is : " << Ct << "\n";

    Omega_rad = sqrt((2*T) / (rho * A * R * R * Ct));

    Omega_rpm = Omega_rad * rads2rpm;

    // std::cout << "Main Ct is : " << (rho * A * R * R * Ct) << "\n";
    // std::cout << "Rotor Rpm is : " << Omega_rpm << "\n";

    

}
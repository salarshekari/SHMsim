#include <iostream>
// #include <vector>
#include "inc/EOM.h"
#include <fstream>

// Omega Trim is : 1812.49 ras/s

#define rads2rpm 9.5492968

int main()
{
    std::ofstream outfile;

    outfile.open("log.csv");

    outfile << "Sim Time" << ",";
    outfile << "X Pos" << ",";
    outfile << "Y Pos" << ",";
    outfile << "Z Pos" << ",";
    outfile << "Phi" << ",";
    outfile << "Theta" << ",";
    outfile << "Psi" << ",";
    outfile << "Height Setpoint" << ",";
    outfile << "Phi Setpoint" << ",";
    outfile << "Theta Setpoint" << ",";
    outfile << "Heading Setpoint" << std::endl;


    double mass = 2.655;         // Weight (Kg)
    double arm_len = 0.254;      // Arm Length (m)
    double Ix = 0.121;           // X Axis moment of inertia (Kg.m^2) 
    double Iy = 0.18;            // Y Axis moment of inertia (Kg.m^2)
    double Iz = 0.293;           // Z Axis moment of inertia (Kg.m^2)
    // double Ct = 0.00000572;     // Thrust Coefficient (N.s^2)   9.62701e-06
    double Ct = 0.00251461;     // Thrust Coefficient (N.s^2)   9.62701e-06
    double Cm = 0.000139709;    // Thrust Coefficient (N.s^2)
    double Jr = 0.0003292;

    double density = 1.225 * 0.03;
    double Blade_Radius_m = 0.4572 / 2; 

    std::vector<double> Om;

    Om.resize(4);

    EOM eomobj(mass, arm_len, Ix, Iy, Iz, Jr, Ct, Cm, Blade_Radius_m, density);

    eomobj.pos_init(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    eomobj.rate_init(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    eomobj.set_alt_gain(0.0412,0.01,0.9854);
    eomobj.set_pitch_gain(0.000945,0.0,0.0542);
    eomobj.set_roll_gain(0.0009,0.0,0.05);
    eomobj.set_yaw_gain(0.004,0.0,0.2);


    // %%%%%%%%%%%%%%%%%%%%%%%%%% Controller Variables %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    double alt_current_state = 0.0;
    double alt_last_state = 0.0;

    double pitch_current_state = 0.0;
    double pitch_last_state = 0.0;

    double roll_current_state = 0.0;
    double roll_last_state = 0.0;

    double psi_current_state = 0.0;
    double psi_last_state = 0.0;

    // %%%%%%%%%%%%%%%%%%%%%%%%%% Controller Setpoints %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    double height_setpoint = 4.0;
    double phi_setpoint = 0.0;
    double theta_setpoint = 0.0;
    double psi_setpoint = 25.0;

    Om[0] = 1812.5;//.999999999;
    Om[1] = 1812.5;//.999999999;
    Om[2] = 1812.5;//.999999999;
    Om[3] = 1812.5;//.999999999;

    eomobj.setMotorSpeed_rad_per_second(Om);

    
    while(1)
    {
        // eomobj.doTrime(Om, 2.1);

        // eomobj.setMotorSpeed_rad_per_second(Om);

        alt_current_state = eomobj.getZ();
        pitch_current_state = eomobj.getTheta();
        roll_current_state = eomobj.getPhi();
        psi_current_state = eomobj.getPsi();



        double alt_pid = eomobj.AltControl(height_setpoint, 0.01, alt_current_state, alt_last_state);

        if(eomobj.getZ() > (height_setpoint-0.02))
        {
            eomobj.YawControl(25, 0.01, psi_current_state, psi_last_state, alt_pid);
            eomobj.PitchControl(0.0, 0.01, pitch_current_state, pitch_last_state, alt_pid);
            eomobj.RollControl(0.0, 0.01, roll_current_state, roll_last_state, alt_pid);
        }

        // if(eomobj.getZ() > (height_setpoint-0.02))
        // {
        //     // eomobj.YawControl(0.0, 0.01, psi_current_state, psi_last_state, alt_pid);
        //     eomobj.PitchControl(2.0, 0.01, pitch_current_state, pitch_last_state, alt_pid);
        //     eomobj.RollControl(0.0, 0.01, roll_current_state, roll_last_state, alt_pid);
        // }

        // eomobj.PitchControl(0.0, 0.01, pitch_current_state, pitch_last_state);
        eomobj.RollControl(0.0, 0.01, roll_current_state, roll_last_state, alt_pid);
        


        eomobj.Run(0.01);

        alt_last_state = alt_current_state;
        pitch_last_state = pitch_current_state;
        roll_last_state = roll_current_state;
        psi_last_state = psi_current_state;

        // std::cout << "Omega1 is: " << Om[0] * rads2rpm << "\t\t";
        // std::cout << "Omega1 (rad/s) is: " << Om[0] << "\t\t";
        // std::cout << "down accel is : " << eomobj.getDownAccel() << "\t";

        std::cout << "Sim Time is : " << eomobj.getSimTime() << "\t\t";
        std::cout << "X pos is: " << eomobj.getX() << "\t\t";
        std::cout << "Y pos is: " << eomobj.getY() << "\t\t";
        std::cout << "Z pos is: " << eomobj.getZ() << "\t\t";
        std::cout << "Theta is: " << eomobj.getTheta() << "\t\t";
        // std::cout << "Phi is : " << eomobj.getPhi() << "\n";
        std::cout << "Psi is : " << eomobj.getPsi() << "\n";



        outfile << eomobj.getSimTime() << ",";
        outfile << eomobj.getX() << ",";
        outfile << eomobj.getY() << ",";
        outfile << eomobj.getZ() << ",";
        outfile << eomobj.getPhi() << ",";
        outfile << eomobj.getTheta() << ",";
        outfile << eomobj.getPsi() << ",";
        outfile << height_setpoint << ",";
        outfile << phi_setpoint << ",";
        outfile << theta_setpoint << ",";
        outfile << psi_setpoint  << std::endl;

        // if(eomobj.getPsi() > psi_setpoint) break;
        if(eomobj.getSimTime() > 500) break;
    }



    return 0;    
}
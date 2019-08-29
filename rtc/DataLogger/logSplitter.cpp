#include <limits>
#include <iomanip>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include "hrpsys/idl/RobotHardwareService.hh"

int main(int argc, char *argv[])
{
    if (argc < 9){
        std::cerr << "Usage: " << argv[0] << " [basename of rstate2 log] [dof] [no. of extra servo states] [no. of force sensors] [no. of 3 axes gyro] [no. of 3 axes accelerometers] [no. of batteries] [no. of thermometers]" << std::endl;
        return 1;
    }

    std::string basename(argv[1]);
    std::ifstream ifs((basename+".rstate2").c_str());
    if (!ifs.is_open()){
        std::cerr << "failed to open " << argv[1] << ".rstate2" << std::endl;
        return 2;
    }
    int dof = atoi(argv[2]);
    int nextrass = atoi(argv[3]);
    int nfsensor = atoi(argv[4]);
    int ngyro = atoi(argv[5]);
    int naccel = atoi(argv[6]);
    int nbattery = atoi(argv[7]);
    int ntemp = atoi(argv[8]);

    std::ofstream ofsq((basename+".q").c_str());
    std::ofstream ofsqref((basename+".qRef").c_str());
    std::ofstream ofstau((basename+".tau").c_str());
    std::ofstream ofsss((basename+".sstate").c_str());
    std::ofstream ofsfsensor((basename+".fsensor").c_str());
    std::ofstream ofsgyro((basename+".gyro").c_str());
    std::ofstream ofsaccel((basename+".accel").c_str());
    std::ofstream ofsbat((basename+".bat").c_str());
    std::ofstream ofstemp((basename+".temp").c_str());

    ofsq.setf(std::ios::fixed, std::ios::floatfield);
    ofsqref.setf(std::ios::fixed, std::ios::floatfield);
    ofstau.setf(std::ios::fixed, std::ios::floatfield);
    ofsss.setf(std::ios::fixed, std::ios::floatfield);
    ofsfsensor.setf(std::ios::fixed, std::ios::floatfield);
    ofsgyro.setf(std::ios::fixed, std::ios::floatfield);
    ofsaccel.setf(std::ios::fixed, std::ios::floatfield);
    ofsbat.setf(std::ios::fixed, std::ios::floatfield);
    ofstemp.setf(std::ios::fixed, std::ios::floatfield);

    double time, v;
    int ss;
    std::string str;
    
    ifs >> time;
    while(!ifs.eof()){
        // q
        ofsq << time << " ";
        for (int i=0; i<dof; i++){
            ifs >> v; ofsq << v << " ";
        }
        ofsq << std::endl;
        // qRef
        ofsqref << time << " ";
        for (int i=0; i<dof; i++){
            ifs >> v; ofsqref << v << " ";
        }
        ofsqref << std::endl;
        // tau
        ofstau << time << " ";
        for (int i=0; i<dof; i++){
            ifs >> v; ofstau << v << " ";
        }
        ofstau << std::endl;
        // servo state
        ofsss << time << " ";
        for (int i=0; i<dof; i++){
            ifs >> ss;
            ofsss << ((ss&OpenHRP::RobotHardwareService::CALIB_STATE_MASK) >> OpenHRP::RobotHardwareService::CALIB_STATE_SHIFT) << " ";
            ofsss << ((ss&OpenHRP::RobotHardwareService::SERVO_STATE_MASK) >> OpenHRP::RobotHardwareService::SERVO_STATE_SHIFT) << " ";
            ofsss << ((ss&OpenHRP::RobotHardwareService::POWER_STATE_MASK) >> OpenHRP::RobotHardwareService::POWER_STATE_SHIFT) << " ";
            ofsss << ((ss&OpenHRP::RobotHardwareService::SERVO_ALARM_MASK) >> OpenHRP::RobotHardwareService::SERVO_ALARM_SHIFT) << " ";
            ofsss << ((ss&OpenHRP::RobotHardwareService::DRIVER_TEMP_MASK) >> OpenHRP::RobotHardwareService::DRIVER_TEMP_SHIFT) << " ";
            for (int j=0; j<nextrass; j++){
                ifs >> ss;
                ofsss << ss << " ";
            }
        }
        ofsss << std::endl;
        // force sensor
        ofsfsensor << time << " ";
        for (int i=0; i<6*nfsensor; i++){
            ifs >> v; ofsfsensor << v << " ";
        }
        ofsfsensor << std::endl;
        // gyro
        ofsgyro << time << " ";
        for (int i=0; i<3*ngyro; i++){
            ifs >> v; ofsgyro << v << " ";
        }
        ofsgyro << std::endl;
        // accelerometer
        ofsaccel << time << " ";
        for (int i=0; i<3*naccel; i++){
            ifs >> v; ofsaccel << v << " ";
        }
        ofsaccel << std::endl;
        // battery
        ofsbat << time << " ";
        for (int i=0; i<3*nbattery+2; i++){
            ifs >> str;
            if (str == "nan"){
                v = std::numeric_limits<double>::quiet_NaN();
            }else{
                v = atof(str.c_str());
            }
            ofsbat << v << " ";
        }
        ofsbat << std::endl;
        // thermometer
        ofstemp << time << " ";
        for (int i=0; i<ntemp; i++){
            ifs >> v; ofstemp << v << " ";
        }
        ofstemp << std::endl;
        
        ifs >> time;
    }
    
    return 0;
}

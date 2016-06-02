/* -*- coding:utf-8-unix; mode:c++; -*- */
#include "ResolvedMomentumControl.h"
#include <rtm/Manager.h>
#include <rtm/CorbaNaming.h>
#include <hrpModel/ModelLoaderUtil.h>
#include <cmath>

inline double deg2rad (double deg) { return deg * M_PI / 180.0; }

int main(int argc, char* argv[])
{
    bool use_gnuplot = true;
    if (argc >= 2) {
        if ( std::string(argv[1])== "--use-gnuplot" ) {
            use_gnuplot = (std::string(argv[2])=="true");
        }
    }

    hrp::BodyPtr m_robot = hrp::BodyPtr(new hrp::Body());
    RTC::Manager& rtcManager = RTC::Manager::instance();
    std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
    int comPos = nameServer.find(",");
    if (comPos < 0){
        comPos = nameServer.length();
    }
    nameServer = nameServer.substr(0, comPos);
    RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());
    if (!loadBodyFromModelLoader(m_robot, "file:///home/ishikawa/ros/indigo/src/rtm-ros-robotics/rtmros_hrp2/jsk_models/CHIDORI/CHIDORImain.wrl",
                                 CosNaming::NamingContext::_duplicate(naming.getRootContext()))) {
        std::cerr << "Failed to load model"  << std::endl;
    }

    // Reset Pose
    m_robot->joint(2)->q = deg2rad(-20);
    m_robot->joint(3)->q = deg2rad(40);
    m_robot->joint(4)->q = deg2rad(-20);
    m_robot->joint(8)->q = deg2rad(40);
    m_robot->joint(9)->q = deg2rad(-20);
    m_robot->joint(10)->q = deg2rad(-20);
    m_robot->calcForwardKinematics();

    // hrp::JointPath jp(m_robot->rootLink(), m_robot->joint(5));
    // std::cerr << "Debug" << std::endl;
    // std::cerr << (m_robot->rootLink()->p).transpose() << std::endl;
    // std::cerr <<  m_robot->totalMass() << std::endl;
    // std::cerr << (m_robot->rootLink()->v).transpose() << ", " <<  (m_robot->rootLink()->w).transpose() << std::endl; // reference
    // std::cerr <<  (m_robot->joint(5)->p).transpose() << std::endl;
    // std::cerr << jp.Jacobian() << std::endl;
    // std::cerr << jp.Jacobian().inverse() << std::endl;

    double dt = 0.01, max_tm = 8.0;

    hrp::dvector6 SVec;
    SVec << 1, 1, 1, 1, 1, 1;
    rats::RMController rmc(m_robot, SVec);
    rmc.addConstraintLink(m_robot, "RLEG_JOINT5");
    rmc.addConstraintLink(m_robot, "LLEG_JOINT5");

    std::string fname("/tmp/plot.dat");
    FILE* fp = fopen(fname.c_str(), "w");
    double tm = 0;
    hrp::Vector3 Pref, Lref, ref_basePos, P, L;
    hrp::Matrix33 ref_baseRot;
    Lref = hrp::Vector3::Zero();
    while (tm < max_tm) {
        // Pref(1) = -3 * cos(tm * M_PI / max_tm * 2);
        // Pref(2) = 3 * sin(tm * M_PI / max_tm * 4);
        Pref(0) = 0;
        Pref(1) = 0;
        Pref(2) = 0.3 * cos(tm * M_PI / max_tm * 2);
        rmc.rmControl(m_robot, Pref, Lref, hrp::dvector6::Zero(6, 1), ref_basePos, ref_baseRot, dt);
        P = m_robot->totalMass() * (ref_basePos - m_robot->rootLink()->p) / dt;
        m_robot->calcForwardKinematics();
        fprintf(fp, "%f %f %f %f %f %f %f %f %f %f\n",
                    tm,
                    Pref(0),
                    P(0),
                    Lref(0),
                    Pref(1),
                    P(1),
                    Lref(1),
                    Pref(2),
                    P(2),
                    Lref(2)
                    );
        tm += dt;
    }

    fclose(fp);
    if (use_gnuplot) {
        FILE* gp[3];
        std::string titles[3] = {"X", "Y", "Z"};
        for (size_t ii = 0; ii < 3; ii++) {
            gp[ii] = popen("gnuplot", "w");
            fprintf(gp[ii], "set title \"%s\"\n", titles[ii].c_str());
            fprintf(gp[ii], "plot \"%s\" using 1:%zu with lines title \"Pref\"\n", fname.c_str(), ( ii * 3 + 2));
            fprintf(gp[ii], "replot \"%s\" using 1:%zu with lines title \"P\"\n", fname.c_str(), ( ii * 3 + 3));
            // fprintf(gp[ii], "replot \"%s\" using 1:%zu with lines title \"refzmp\"\n", fname.c_str(), ( ii * 3 + 4));
            fflush(gp[ii]);
        }
        double tmp;
        std::cin >> tmp;
        for (size_t j = 0; j < 3; j++) pclose(gp[j]);
    }
    return 0;
}

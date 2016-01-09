// -*- C++ -*-
/*!
 * @file JointPathExC.cpp
 * @brief Standalone component
 * @date $Date$
 *
 * $Id$
 */

#include <rtm/Manager.h>
#include <rtm/CorbaNaming.h>
#include <hrpModel/Body.h>
#include <hrpModel/Link.h>
#include <hrpModel/Sensor.h>
#include <hrpModel/ModelLoaderUtil.h>
#include "JointPathEx.h"


extern "C" {
    hrp::JointPathExPtr jpe;
    hrp::BodyPtr m_robot(new hrp::Body());
    static std::string print_prefix("[jpe]");

    int initializeOpenHRPModel (char* _filename)
    {
        int rtmargc=0;
        char** argv=NULL;
        //std::vector<char *> rtmargv;
        //rtmargv.push_back(argv[0]);
        rtmargc++;

        RTC::Manager* manager;
        //manager = RTC::Manager::init(rtmargc, rtmargv.data());
        manager = RTC::Manager::init(rtmargc, argv);

        std::string nameServer = manager->getConfig()["corba.nameservers"];
        int comPos = nameServer.find(",");
        if (comPos < 0){
            comPos = nameServer.length();
        }
        nameServer = nameServer.substr(0, comPos);
        RTC::CorbaNaming naming(manager->getORB(), nameServer.c_str());
        std::string filename(_filename);
        if (!loadBodyFromModelLoader(m_robot, filename.c_str(),
                                     CosNaming::NamingContext::_duplicate(naming.getRootContext()),
                                     true)){
            std::cerr << print_prefix << " Failed to load model[" << filename << "]" << std::endl;
            return 1;
        } else {
            std::cerr << print_prefix << " Success to load model[" << filename << "]" << std::endl;
        }
        return 0;
    };

    int initializeJointPathExInstance (char* root_link_name, char* target_link_name)
    {
        jpe = hrp::JointPathExPtr(new hrp::JointPathEx(m_robot, m_robot->link(root_link_name), m_robot->link(target_link_name), 0.002, false, std::string("test")));
        if ( !jpe ) {
            std::cerr << print_prefix << " Fail to joint path from " << root_link_name << " to " << target_link_name << std::endl;
        } else {
            std::cerr << print_prefix << " Success to joint path from " << root_link_name << " to " << target_link_name << " (dof = " << jpe->numJoints() << std::endl;
        }
        return 0;
    }

    int _setJointAngles (double* ja)
    {
        for ( int i = 0; i < m_robot->numJoints(); i++ ) {
            m_robot->joint(i)->q = ja[i];
        }
        m_robot->calcForwardKinematics();
        return 0;
    }

    int _getJointAngles (double* ja)
    {
        for ( int i = 0; i < m_robot->numJoints(); i++ ) {
            ja[i] = m_robot->joint(i)->q;
        }
    }

    int _calcInverseKinematics2Loop (double* _vel_p, double* _vel_r)
    {
        hrp::dvector qrefv = hrp::dvector::Zero(jpe->numJoints());
        hrp::Vector3 vel_p, vel_r;
        for (size_t i = 0; i < 3; i++) {
            vel_p[i] = _vel_p[i];
            vel_r[i] = _vel_r[i];
        }
        jpe->calcInverseKinematics2Loop(vel_p, vel_r, 1.0, 0.002, 0.0, &qrefv);
        return 0;
    }
};

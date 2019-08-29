// -*- C++ -*-
/*!
 * @file  Simulator.cpp
 * @brief dynamics simulator component
 * $Date$
 *
 * $Id$
 */

#include <rtm/CorbaNaming.h>

#include "Simulator.h"
#include <hrpCorba/OpenHRPCommon.hh>
#include <hrpModel/ModelLoaderUtil.h>
#include <hrpModel/OnlineViewerUtil.h>
#include <hrpModel/Link.h>
#include "hrpsys/util/Project.h"

// Module specification
// <rtc-template block="module_spec">
static const char* component_spec[] =
{
    "implementation_id", "Simulator",
    "type_name",         "Simulator",
    "description",       "dynamics simulator component",
    "version",           HRPSYS_PACKAGE_VERSION,
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.project", "",
    "conf.default.kinematics_only", "0",
    "conf.default.useOLV", "0",
    ""
};
// </rtc-template>

Simulator::Simulator(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      // <rtc-template block="initializer">
      m_sceneStateOut("state", m_sceneState),
      // </rtc-template>
      dummy(0)
{
}

Simulator::~Simulator()
{
}



RTC::ReturnCode_t Simulator::onInitialize()
{
    std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
    // <rtc-template block="bind_config">
    // Bind variables and configuration variable
    bindParameter("project", m_project, "");  
    bindParameter("kinematics_only", m_kinematicsOnly, "0");  
    bindParameter("useOLV", m_useOLV, "0");  
  
    // </rtc-template>

    // Registration: InPort/OutPort/Service
    // <rtc-template block="registration">
    // Set InPort buffers

    // Set OutPort buffer
    addOutPort("state", m_sceneStateOut);
  
    // Set service provider to Ports
  
    // Set service consumers to Ports
  
    // Set CORBA Service Ports
  
    // </rtc-template>

    //RTC::Properties& prop = getProperties();

    return RTC::RTC_OK;
}



/*
  RTC::ReturnCode_t Simulator::onFinalize()
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t Simulator::onStartup(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t Simulator::onShutdown(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

RTC::ReturnCode_t Simulator::onActivated(RTC::UniqueId ec_id)
{
    std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;

    if (m_project == ""){
        std::cerr << "Project file is not specified." << std::endl;
        return RTC::RTC_ERROR;
    }

    Project prj;
    if (!prj.parse(m_project)) return RTC::RTC_ERROR;

    if ( m_kinematicsOnly == false ) {
	m_kinematicsOnly = prj.kinematicsOnly();
    }
    std::cout << "kinematics_only : " << m_kinematicsOnly << std::endl;

    m_world.clearBodies();
    m_world.constraintForceSolver.clearCollisionCheckLinkPairs();
    m_world.setCurrentTime(0.0);
    m_world.setTimeStep(prj.timeStep());
    std::cout << "time step = " << prj.timeStep() << std::endl;
    if(prj.isEuler()){
        m_world.setEulerMethod();
    } else {
        m_world.setRungeKuttaMethod();
    }

    RTC::Manager& rtcManager = RTC::Manager::instance();
    std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
    int comPos = nameServer.find(",");
    if (comPos < 0){
        comPos = nameServer.length();
    }
    nameServer = nameServer.substr(0, comPos);
    RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());

    std::cout << "m_useOLV:" << m_useOLV << std::endl;
    if (m_useOLV){
        m_olv = hrp::getOnlineViewer(CosNaming::NamingContext::_duplicate(naming.getRootContext()));
    }

    for (std::map<std::string, ModelItem>::iterator it=prj.models().begin();
         it != prj.models().end(); it++){
        RTCBodyPtr body(new RTCBody());
        if (!loadBodyFromModelLoader(body, it->second.url.c_str(), 
                                     CosNaming::NamingContext::_duplicate(naming.getRootContext()),
                                     true)){
            std::cerr << "failed to load model[" << it->second.url << "]" << std::endl;
        }else{
            body->setName(it->first);
            for (std::map<std::string, JointItem>::iterator it2=it->second.joint.begin();
                 it2 != it->second.joint.end(); it2++){
                hrp::Link *link = body->link(it2->first);
                if (link) link->isHighGainMode = it2->second.isHighGain;
            }
            m_world.addBody(body);
            body->createPorts(this);
            m_bodies.push_back(body);
        }
        if (m_useOLV){
            m_olv->load(it->first.c_str(), it->second.url.c_str());
        }
    }
    if (m_useOLV){
        m_olv->clearLog();
        initWorldState(m_state, m_world); 
    }


    for (unsigned int i=0; i<prj.collisionPairs().size(); i++){
        const CollisionPairItem &cpi = prj.collisionPairs()[i];
        int bodyIndex1 = m_world.bodyIndex(cpi.objectName1);
        int bodyIndex2 = m_world.bodyIndex(cpi.objectName2);

        if(bodyIndex1 >= 0 && bodyIndex2 >= 0){
            hrp::BodyPtr bodyPtr1 = m_world.body(bodyIndex1);
            hrp::BodyPtr bodyPtr2 = m_world.body(bodyIndex2);

            std::vector<hrp::Link*> links1;
            if(cpi.jointName1.empty()){
                const hrp::LinkTraverse& traverse = bodyPtr1->linkTraverse();
                links1.resize(traverse.numLinks());
                std::copy(traverse.begin(), traverse.end(), links1.begin());
            } else {
                links1.push_back(bodyPtr1->link(cpi.jointName1));
            }

            std::vector<hrp::Link*> links2;
            if(cpi.jointName2.empty()){
                const hrp::LinkTraverse& traverse = bodyPtr2->linkTraverse();
                links2.resize(traverse.numLinks());
                std::copy(traverse.begin(), traverse.end(), links2.begin());
            } else {
                links2.push_back(bodyPtr2->link(cpi.jointName2));
            }

            for(size_t j=0; j < links1.size(); ++j){
                for(size_t k=0; k < links2.size(); ++k){
                    hrp::Link* link1 = links1[j];
                    hrp::Link* link2 = links2[k];

                    if(link1 && link2 && link1 != link2){
                        m_world.constraintForceSolver.addCollisionCheckLinkPair
                            (bodyIndex1, link1, bodyIndex2, link2, 
                             cpi.staticFriction, cpi.slidingFriction, 0.01, 0.0, 0.0);
                    }
                }
            }
        }
    }

    m_world.enableSensors(false);
  
    int nBodies = m_world.numBodies();
    for(int i=0; i < nBodies; ++i){
        hrp::BodyPtr bodyPtr = m_world.body(i);
        bodyPtr->initializeConfiguration();
    }

    for (std::map<std::string, ModelItem>::iterator it=prj.models().begin();
         it != prj.models().end(); it++){
        hrp::BodyPtr body = m_world.body(it->first);
        for (std::map<std::string, JointItem>::iterator it2=it->second.joint.begin();
             it2 != it->second.joint.end(); it2++){
            hrp::Link *link = body->link(it2->first);
            if (!link) continue;
            if (link->isRoot()){
                link->p = it2->second.translation;
                link->setAttitude(it2->second.rotation);
            }else{
                link->q = it2->second.angle;
            }
        }
        body->calcForwardKinematics();
    }
  
    m_world.initialize();
    m_world.constraintForceSolver.useBuiltinCollisionDetector(true);
    m_world.constraintForceSolver.enableConstraintForceOutput(true);
  
    m_sceneState.states.length(m_bodies.size());
    for (unsigned int i=0; i<m_bodies.size(); i++){
        m_sceneState.states[i].name = CORBA::string_dup(m_bodies[i]->name().c_str());
        m_sceneState.states[i].q.length(m_bodies[i]->numJoints());
    }


    return RTC::RTC_OK;
}

RTC::ReturnCode_t Simulator::onDeactivated(RTC::UniqueId ec_id)
{
    std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
    return RTC::RTC_OK;
}

RTC::ReturnCode_t Simulator::onExecute(RTC::UniqueId ec_id)
{
    //std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << std::endl;
    // output current state
    m_sceneState.time = m_world.currentTime();
    for (unsigned int i=0; i<m_bodies.size(); i++){
        m_bodies[i]->output(m_sceneState.states[i]);
    }
    m_sceneStateOut.write();

    // input command
    for (unsigned int i=0; i<m_bodies.size(); i++) m_bodies[i]->input();

    if (m_kinematicsOnly){
        for(unsigned int i=0; i < m_world.numBodies(); ++i){
            m_world.body(i)->calcForwardKinematics();
        }
        m_world.setCurrentTime(m_world.currentTime() + m_world.timeStep());
    }else{
        m_world.constraintForceSolver.clearExternalForces();
    
        OpenHRP::CollisionSequence collision;
        m_world.calcNextState(collision);
    }

    if (m_useOLV){
        getWorldState(m_state, m_world);
        m_olv->update( m_state );
    }
    return RTC::RTC_OK;
}

/*
  RTC::ReturnCode_t Simulator::onAborting(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t Simulator::onError(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t Simulator::onReset(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t Simulator::onStateUpdate(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t Simulator::onRateChanged(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/



extern "C"
{

    void SimulatorInit(RTC::Manager* manager)
    {
        RTC::Properties profile(component_spec);
        manager->registerFactory(profile,
                                 RTC::Create<Simulator>,
                                 RTC::Delete<Simulator>);
    }

};



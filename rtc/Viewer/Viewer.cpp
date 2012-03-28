// -*- C++ -*-
/*!
 * @file  Viewer.cpp
 * @brief viewer component
 * $Date$
 *
 * $Id$
 */

#include <rtm/CorbaNaming.h>

#include "Project.h"
#include "IrrModel.h"
#include "RTCGLbody.h"
#include "Viewer.h"

// Module specification
// <rtc-template block="module_spec">
static const char* component_spec[] =
{
    "implementation_id", "Viewer",
    "type_name",         "Viewer",
    "description",       "viewer component",
    "version",           "1.0",
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.project", "",
    ""
};
// </rtc-template>

Viewer::Viewer(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      // <rtc-template block="initializer">
      m_sceneStateIn("state", m_sceneState),
      // </rtc-template>
      dummy(0)
{
    m_scene = new GLscene();
}

Viewer::~Viewer()
{
}



RTC::ReturnCode_t Viewer::onInitialize()
{
    std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
    // <rtc-template block="bind_config">
    // Bind variables and configuration variable
    bindParameter("project", m_project, "");  
  
    // </rtc-template>

    // Registration: InPort/OutPort/Service
    // <rtc-template block="registration">
    // Set InPort buffers
    addInPort("state", m_sceneStateIn);

    // Set OutPort buffer
  
    // Set service provider to Ports
  
    // Set service consumers to Ports
  
    // Set CORBA Service Ports
  
    // </rtc-template>

    //RTC::Properties& prop = getProperties();

    return RTC::RTC_OK;
}



/*
  RTC::ReturnCode_t Viewer::onFinalize()
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t Viewer::onStartup(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t Viewer::onShutdown(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

RTC::ReturnCode_t Viewer::onActivated(RTC::UniqueId ec_id)
{
    std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;

    RTC::Manager& rtcManager = RTC::Manager::instance();
    std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
    int comPos = nameServer.find(",");
    if (comPos < 0){
        comPos = nameServer.length();
    }
    nameServer = nameServer.substr(0, comPos);
    RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());

    if (m_project == ""){
        std::cerr << "Project file is not specified." << std::endl;
        return RTC::RTC_ERROR;
    }

    Project prj;
    if (!prj.parse(m_project)) return RTC::RTC_ERROR;

    m_scene->init();

    for (std::map<std::string, ModelItem>::iterator it=prj.models().begin();
         it != prj.models().end(); it++){
        OpenHRP::BodyInfo_var binfo;
        binfo = hrp::loadBodyInfo(it->second.url.c_str(),
                                  CosNaming::NamingContext::_duplicate(naming.getRootContext()));
        if (CORBA::is_nil(binfo)){
            std::cerr << "failed to load model[" << it->second.url.c_str() << "]" 
                      << std::endl;
        }else{
            m_bodies[it->first] = new RTCGLbody(m_scene->addBody(binfo), this);
        }
    }

    return RTC::RTC_OK;
}

RTC::ReturnCode_t Viewer::onDeactivated(RTC::UniqueId ec_id)
{
    std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;

    for (std::map<std::string, RTCGLbody *>::iterator it = m_bodies.begin();
         it != m_bodies.end(); it++){
        delete it->second;
    }
    m_bodies.clear();

    return RTC::RTC_OK;
}

RTC::ReturnCode_t Viewer::onExecute(RTC::UniqueId ec_id)
{
    //std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << std::endl;

    if (m_sceneStateIn.isNew()){
        do{
            m_sceneStateIn.read();
        }while(m_sceneStateIn.isNew());
        for (unsigned int i=0; i<m_sceneState.states.length(); i++){
            const OpenHRP::RobotState& state = m_sceneState.states[i];
            std::string name(state.name);
            RTCGLbody *rtcglb=m_bodies[name];
            if (rtcglb){
                GLbody *body = rtcglb->body();
                body->setPosition(state.basePose.position.x,
                                  state.basePose.position.y,
                                  state.basePose.position.z);
                body->setOrientation(state.basePose.orientation.r,
                                     state.basePose.orientation.p,
                                     state.basePose.orientation.y);
                body->setPosture(state.q.get_buffer());
            }
        }
    }

    for (std::map<std::string, RTCGLbody *>::iterator it=m_bodies.begin();
         it != m_bodies.end(); it++){
        it->second->input();
    }

    m_scene->draw();

    return RTC::RTC_OK;
}

/*
  RTC::ReturnCode_t Viewer::onAborting(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t Viewer::onError(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t Viewer::onReset(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t Viewer::onStateUpdate(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t Viewer::onRateChanged(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/



extern "C"
{

    void ViewerInit(RTC::Manager* manager)
    {
        RTC::Properties profile(component_spec);
        manager->registerFactory(profile,
                                 RTC::Create<Viewer>,
                                 RTC::Delete<Viewer>);
    }

};



// -*- C++ -*-
/*!
 * @file  Viewer.cpp
 * @brief viewer component
 * $Date$
 *
 * $Id$
 */

#include <rtm/CorbaNaming.h>

#include "hrpsys/util/Project.h"
#include <hrpModel/ModelLoaderUtil.h>
#include "hrpsys/util/GLbody.h"
#include "hrpsys/util/GLlink.h"
#include "hrpsys/util/GLutil.h"
#include "GLscene.h"
#include "hrpsys/idl/HRPDataTypes.hh"
#include "RTCGLbody.h"
#include "Viewer.h"

// Module specification
// <rtc-template block="module_spec">
static const char* component_spec[] =
{
    "implementation_id", "Viewer",
    "type_name",         "Viewer",
    "description",       "viewer component",
    "version",           HRPSYS_PACKAGE_VERSION,
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
      m_scene(&m_log),
      m_window(&m_scene, &m_log),
      dummy(0)
{
    m_log.enableRingBuffer(1);
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

    m_window.init();

    for (std::map<std::string, ModelItem>::iterator it=prj.models().begin();
         it != prj.models().end(); it++){
        OpenHRP::BodyInfo_var binfo;
        binfo = hrp::loadBodyInfo(it->second.url.c_str(),
                                  CosNaming::NamingContext::_duplicate(naming.getRootContext()));
        if (CORBA::is_nil(binfo)){
            std::cerr << "failed to load model[" << it->second.url.c_str() << "]" 
                      << std::endl;
        }else{
            GLbody *glbody = new GLbody();
            hrp::BodyPtr body(glbody);
            hrp::loadBodyFromBodyInfo(body, binfo, false, GLlinkFactory);
            loadShapeFromBodyInfo(glbody, binfo);
            body->setName(it->first);
            m_scene.WorldBase::addBody(body);
            m_bodies[it->first] = new RTCGLbody(glbody, this);
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
        m_log.add(m_sceneState);
    }

    for (std::map<std::string, RTCGLbody *>::iterator it=m_bodies.begin();
         it != m_bodies.end(); it++){
        it->second->input();
    }

    m_window.processEvents();
    m_window.draw();
    m_window.swapBuffers();

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



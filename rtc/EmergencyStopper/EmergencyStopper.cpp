// -*- C++ -*-
/*!
 * @file  EmergencyStopper.cpp
 * @brief emergency stopper
 * $Date$
 *
 * $Id$
 */

#include "EmergencyStopper.h"
#include "util/VectorConvert.h"

// Module specification
// <rtc-template block="module_spec">
static const char* emergencystopper_spec[] =
  {
    "implementation_id", "EmergencyStopper",
    "type_name",         "EmergencyStopper",
    "description",       "emergency stopper",
    "version",           HRPSYS_PACKAGE_VERSION,
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.string", "test",
    "conf.default.intvec", "1,2,3",
    "conf.default.double", "1.234",

    ""
  };
// </rtc-template>

EmergencyStopper::EmergencyStopper(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_dataIn("dataIn", m_data),
    m_dataOut("dataOut", m_data),
    m_EmergencyStopperServicePort("EmergencyStopperService"),
    // </rtc-template>
	dummy(0)
{
  std::cout << "EmergencyStopper::EmergencyStopper()" << std::endl;
  m_data.data = 0;
}

EmergencyStopper::~EmergencyStopper()
{
  std::cout << "EmergencyStopper::~EmergencyStopper()" << std::endl;
}



RTC::ReturnCode_t EmergencyStopper::onInitialize()
{
  std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("string", confstring, "testtest");
  bindParameter("intvec", confintvec, "4,5,6,7");
  bindParameter("double", confdouble, "4.567");
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("dataIn", m_dataIn);

  // Set OutPort buffer
  addOutPort("dataOut", m_dataOut);
  
  // Set service provider to Ports
  m_EmergencyStopperServicePort.registerProvider("service0", "EmergencyStopperService", m_EmergencyStopperService);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_EmergencyStopperServicePort);
  
  // </rtc-template>

  RTC::Properties& prop = getProperties();
  std::cout << "prop[\"testconf\"] = " << prop["testconf"] << std::endl;

  return RTC::RTC_OK;
}



/*
RTC::ReturnCode_t EmergencyStopper::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t EmergencyStopper::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t EmergencyStopper::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t EmergencyStopper::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t EmergencyStopper::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t EmergencyStopper::onExecute(RTC::UniqueId ec_id)
{
  // std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << "), data = " << m_data.data << std::endl;
  // std::cout << "confstring = " << confstring << std::endl;
  // std::cout << "confintvec = ";
  // for (unsigned int i=0; i<confintvec.size(); i++){
  //     std::cout << confintvec[i] << " ";
  // }
  // std::cout << std::endl;
  // std::cout << "confdouble = " << confdouble << std::endl;

  while (m_dataIn.isNew()){
      m_dataIn.read();
      std::cout << m_profile.instance_name << ": read(), data = " << m_data.data << std::endl;
  }
  m_data.data += 1;

  m_dataOut.write();
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t EmergencyStopper::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t EmergencyStopper::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t EmergencyStopper::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t EmergencyStopper::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t EmergencyStopper::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{

  void EmergencyStopperInit(RTC::Manager* manager)
  {
    RTC::Properties profile(emergencystopper_spec);
    manager->registerFactory(profile,
                             RTC::Create<EmergencyStopper>,
                             RTC::Delete<EmergencyStopper>);
  }

};



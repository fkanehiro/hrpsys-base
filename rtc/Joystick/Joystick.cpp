// -*- C++ -*-
/*!
 * @file  Joystick.cpp
 * @brief Access a joystick control device.
 * @date $Date$
 *
 * $Id$
 */

#include "Joystick.h"
#include "js.h"

// Module specification
// <rtc-template block="module_spec">
static const char* joystick_spec[] =
  {
    "implementation_id", "Joystick",
    "type_name",         "Joystick",
    "description",       "Access a joystick control device.",
    "version",           HRPSYS_PACKAGE_VERSION,
    "vendor",            "AIST",
    "category",          "Human input",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.device", "/dev/input/js1",
    "conf.default.debugLevel", "0",
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
Joystick::Joystick(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_axesOut("Axes", m_axes),
    m_buttonsOut("Buttons", m_buttons),
    m_debugLevel(0)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
Joystick::~Joystick()
{
}



RTC::ReturnCode_t Joystick::onInitialize()
{
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("device", m_device, "/dev/input/js1");
  bindParameter("debugLevel", m_debugLevel, "0");

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  
  // Set OutPort buffer
  addOutPort("Axes", m_axesOut);
  addOutPort("Buttons", m_buttonsOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t Joystick::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Joystick::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Joystick::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t Joystick::onActivated(RTC::UniqueId ec_id)
{
  std::cout << "Joystick::onActivated(" << ec_id << ")" << std::endl;
  m_js = new joystick(m_device.c_str());
  if (m_js->is_open()){
    m_axes.data.length(m_js->nAxes());
    for (unsigned int i=0; i<m_js->nAxes(); i++){
        m_axes.data[i] = m_js->getAxisState(i);
    }
    m_buttons.data.length(m_js->nButtons());
    for (unsigned int i=0; i<m_js->nButtons(); i++){
        m_buttons.data[i] = m_js->getButtonState(i);
    }
    return RTC::RTC_OK;
  }else{
    std::cerr << "Joystick device(" << m_device << ") is not opened" << std::endl;
    return RTC::RTC_ERROR;  
  }
}


RTC::ReturnCode_t Joystick::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << "Joystick::onDeactivated(" << ec_id << ")" << std::endl;
  delete m_js;
  return RTC::RTC_OK;
}



RTC::ReturnCode_t Joystick::onExecute(RTC::UniqueId ec_id)
{
  while(m_js->readEvent());
  if (m_debugLevel > 0) printf("axes:");
  for (unsigned int i=0; i<m_js->nAxes(); i++){
    m_axes.data[i] = m_js->getAxisState(i);
    if (m_debugLevel > 0) printf("%4.1f ", m_axes.data[i]);
  }
  if (m_debugLevel > 0) printf(", buttons:");
  for (unsigned int i=0; i<m_js->nButtons(); i++){
    m_buttons.data[i] = m_js->getButtonState(i);
    if (m_debugLevel > 0) printf("%d", m_buttons.data[i]);
  }
  if (m_debugLevel > 0) printf("\n");

  m_axesOut.write();
  m_buttonsOut.write();

  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t Joystick::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Joystick::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Joystick::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Joystick::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t Joystick::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void JoystickInit(RTC::Manager* manager)
  {
    coil::Properties profile(joystick_spec);
    manager->registerFactory(profile,
                             RTC::Create<Joystick>,
                             RTC::Delete<Joystick>);
  }
  
};



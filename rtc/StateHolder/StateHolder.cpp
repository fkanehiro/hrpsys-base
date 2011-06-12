// -*- C++ -*-
/*!
 * @file  StateHolder.cpp
 * @brief state holder component
 * $Date$
 *
 * $Id$
 */

#include "StateHolder.h"
#include <hrpUtil/Tvmet3d.h>

// Module specification
// <rtc-template block="module_spec">
static const char* stateholder_spec[] =
  {
    "implementation_id", "StateHolder",
    "type_name",         "StateHolder",
    "description",       "state holder",
    "version",           "1.0",
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables

    ""
  };
// </rtc-template>

StateHolder::StateHolder(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_currentQIn("currentQIn", m_currentQ),
    m_qIn("qIn", m_q),
    m_basePosIn("basePosIn", m_basePos),
    m_baseRpyIn("baseRpyIn", m_baseRpy),
    m_qOut("qOut", m_q),
    m_basePosOut("basePosOut", m_basePos),
    m_baseRpyOut("baseRpyOut", m_baseRpy),
    m_baseTformOut("baseTformOut", m_baseTform),
    m_basePoseOut("basePoseOut", m_basePose),
    m_StateHolderServicePort("StateHolderService"),
    m_TimeKeeperServicePort("TimeKeeperService"),
    // </rtc-template>
    dummy(0), m_timeCount(0)
{

  m_service0.setComponent(this);
  m_service1.setComponent(this);
  sem_init(&m_waitSem, 0, 0);
  sem_init(&m_timeSem, 0, 0);
  m_requestGoActual = false;

  m_basePos.data.x = m_basePos.data.y = m_basePos.data.z = 0.0;
  m_baseRpy.data.r = m_baseRpy.data.p = m_baseRpy.data.y = 0.0;
  m_baseTform.data.length(12);
  for (int i=0; i<12; i++) m_baseTform.data[i] = 0.0;
  m_baseTform.data[3] = m_baseTform.data[7] = m_baseTform.data[11] = 0.0;
  m_basePose.data.position.x = 0;
  m_basePose.data.position.y = 0;
  m_basePose.data.position.z = 0;
  m_basePose.data.orientation.r = 0;
  m_basePose.data.orientation.p = 0;
  m_basePose.data.orientation.y = 0;
}

StateHolder::~StateHolder()
{
}



RTC::ReturnCode_t StateHolder::onInitialize()
{
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
    addInPort("currentQIn", m_currentQIn);
    addInPort("qIn", m_qIn);
    addInPort("basePosIn", m_basePosIn);
    addInPort("baseRpyIn", m_baseRpyIn);
  
  // Set OutPort buffer
    addOutPort("qOut", m_qOut);
    addOutPort("basePosOut", m_basePosOut);
    addOutPort("baseRpyOut", m_baseRpyOut);
    addOutPort("baseTformOut", m_baseTformOut);
    addOutPort("basePoseOut", m_basePoseOut);
  
  // Set service provider to Ports
  m_StateHolderServicePort.registerProvider("service0", "StateHolderService", m_service0);
  m_TimeKeeperServicePort.registerProvider("service1", "TimeKeeperService", m_service1);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_StateHolderServicePort);
  addPort(m_TimeKeeperServicePort);

  // </rtc-template>

  RTC::Properties& prop = getProperties();
  coil::stringTo(m_dt, prop["dt"].c_str());
  std::cout << "StateHolder: dt = " << m_dt << std::endl;

  return RTC::RTC_OK;
}



/*
RTC::ReturnCode_t StateHolder::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t StateHolder::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t StateHolder::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t StateHolder::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t StateHolder::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t StateHolder::onExecute(RTC::UniqueId ec_id)
{
    //std::cout << "StateHolder::onExecute(" << ec_id << ")" << std::endl;
    if (m_currentQIn.isNew()){
        m_currentQIn.read();
    }

    if (m_qIn.isNew()){
        m_qIn.read();
    }
    if (m_requestGoActual || (m_q.data.length() == 0 && m_currentQ.data.length() > 0)){
        m_q = m_currentQ;
    }

    if (m_q.data.length() > 0){
        m_qOut.write();
    }

    if (m_requestGoActual){
        m_requestGoActual = false;
        sem_post(&m_waitSem); 
    }

    if (m_basePosIn.isNew()){
        m_basePosIn.read();
    }
    m_basePosOut.write();

    if (m_baseRpyIn.isNew()){
        m_baseRpyIn.read();
    }
    m_baseRpyOut.write();

    double *a = m_baseTform.data.get_buffer();
    a[0] = m_basePos.data.x;
    a[1] = m_basePos.data.y;
    a[2] = m_basePos.data.z;
    hrp::Matrix33 R = hrp::rotFromRpy(m_baseRpy.data.r, 
                                      m_baseRpy.data.p, 
                                      m_baseRpy.data.y); 
    hrp::setMatrix33ToRowMajorArray(R, a, 3);
    m_baseTformOut.write();

    m_basePose.data.position = m_basePos.data;
    m_basePose.data.orientation = m_baseRpy.data;
    m_basePoseOut.write();

    if (m_timeCount > 0){
        m_timeCount--;
        if (m_timeCount == 0) sem_post(&m_timeSem);
    }

    return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t StateHolder::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t StateHolder::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t StateHolder::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t StateHolder::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t StateHolder::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


void StateHolder::goActual()
{
    std::cout << "StateHolder::goActual()" << std::endl;
    m_requestGoActual = true;
    sem_wait(&m_waitSem);
}

void StateHolder::getCommand(StateHolderService::Command &com)
{
    com.jointRefs.length(m_q.data.length());
    memcpy(com.jointRefs.get_buffer(), m_q.data.get_buffer(), sizeof(double)*m_q.data.length());
    com.baseTransform.length(12);
    com.baseTransform[0] = m_basePos.data.x;
    com.baseTransform[1] = m_basePos.data.y;
    com.baseTransform[2] = m_basePos.data.z;
    hrp::Matrix33 R = hrp::rotFromRpy(m_baseRpy.data.r, m_baseRpy.data.p, m_baseRpy.data.y);
    double *a = com.baseTransform.get_buffer();
    hrp::setMatrix33ToRowMajorArray(R, a, 3);
}

void StateHolder::wait(CORBA::Double tm)
{
    m_timeCount = tm/m_dt;
    sem_wait(&m_timeSem);
}
 
extern "C"
{

  void StateHolderInit(RTC::Manager* manager)
  {
    RTC::Properties profile(stateholder_spec);
    manager->registerFactory(profile,
                             RTC::Create<StateHolder>,
                             RTC::Delete<StateHolder>);
  }

};



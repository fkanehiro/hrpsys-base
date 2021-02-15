// -*- C++ -*-
/*!
 * @file  ModifiedServo.cpp
 * @brief ModifiedServo component
 * $Date$ 
 *
 * $Id$ 
 */

#include "ModifiedServo.h"
// #include <iostream>  // Added by Rafa

// Module specification
// <rtc-template block="module_spec">
static const char* modifiedservo_spec[] =
  {
    "implementation_id", "ModifiedServo",
    "type_name",         "ModifiedServo",
    "description",       "ModifiedServo component",
    "version",           HRPSYS_PACKAGE_VERSION,
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

ModifiedServo::ModifiedServo(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_tauRefIn("tauRef", m_tauRef),
    m_qRefIn("qRef", m_qRef),
    m_qIn("q", m_q),
    m_torqueModeIn("torqueMode", m_torqueMode),
    m_tauOut("tau", m_tau),
    m_pgainsIn("pgainsSet", m_pgains),
    m_dgainsIn("dgainsSet", m_dgains),
    m_pgainsOut("pgainsGet", m_pgains),
    m_dgainsOut("dgainsGet", m_dgains),
    // </rtc-template>
    m_gain_fname(""),
    m_dt(0.005),
    m_dof(0)
{
}

ModifiedServo::~ModifiedServo()
{
}


RTC::ReturnCode_t ModifiedServo::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("tauRef", m_tauRefIn);
  addInPort("qRef", m_qRefIn);
  addInPort("q", m_qIn);
  addInPort("torqueMode", m_torqueModeIn);
  addInPort("pgainsSet", m_pgainsIn);
  addInPort("dgainsSet", m_dgainsIn);

  // Set OutPort buffer
  addOutPort("tau", m_tauOut);
  addOutPort("pgainsGet", m_pgainsOut);
  addOutPort("dgainsGet", m_dgainsOut);

  // Set service provider to Ports

  // Set service consumers to Ports

  // Set CORBA Service Ports

  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable

  // </rtc-template>

  std::cout << m_profile.instance_name << ": onInitialize() " << std::endl;

  RTC::Properties & prop = getProperties();

  coil::stringTo(m_dt, prop["dt"].c_str());
  coil::stringTo(m_ref_dt, prop["ref_dt"].c_str());
  m_nstep = m_ref_dt / m_dt;
  m_step = m_nstep;

  m_robot = hrp::BodyPtr(new hrp::Body());

  RTC::Manager & rtcManager = RTC::Manager::instance();
  std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
  int comPos = nameServer.find(",");

  if (comPos < 0)
    comPos = nameServer.length();

  // In case there is more than one, retrieves only the first one
  nameServer = nameServer.substr(0, comPos);

  RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());

  if (!loadBodyFromModelLoader(m_robot, prop["model"].c_str(),
                               CosNaming::NamingContext::_duplicate(naming.getRootContext())))
      std::cerr << "[" << m_profile.instance_name << "] failed to load model "
                << "[" << prop["model"] << "]" << std::endl;
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t ModifiedServo::onFinalize()
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t ModifiedServo::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t ModifiedServo::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t ModifiedServo::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name << ": on Activated" << std::endl;

  if (m_qIn.isNew()) {
    m_qIn.read();
    if (m_dof == 0) {
      m_dof = m_q.data.length();
      readGainFile();
    }
  }

  m_q_old.resize(m_dof);
  m_qRef_old.resize(m_dof);

  m_tauRef.data.length(m_dof);
  m_qRef.data.length(m_dof);
  m_torqueMode.data.length(m_dof);
  
  m_tau.data.length(m_dof);

  m_pgains.data.length(dof);
  m_dgains.data.length(dof);

  for (size_t i = 0; i < dof; i++) {
=======
  for (size_t i = 0; i < m_dof; i++) {
>>>>>>> Used m_ for the member variables of ModifiedServo
    m_tauRef.data[i] = 0.0;
    m_qRef.data[i] = m_qRef_old[i] = m_q_old[i] = m_q.data[i];
    m_torqueMode.data[i] = false;
    m_pgains.data[i] = Pgain[i];
    m_dgains.data[i] = Dgain[i];
  }
  
  return RTC::RTC_OK;
}

RTC::ReturnCode_t ModifiedServo::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name << ": on Deactivated" << std::endl;
  
  return RTC::RTC_OK;
}

RTC::ReturnCode_t ModifiedServo::onExecute(RTC::UniqueId ec_id)
{
  if (m_tauRefIn.isNew())
    m_tauRefIn.read();
  
  if (m_qIn.isNew())
    m_qIn.read();

  if (m_qRefIn.isNew()) {
    m_qRefIn.read();
    m_step = m_nstep;
  }
  if(m_pgainsIn.isNew()){
    m_pgainsIn.read();
  }
  if(m_dgainsIn.isNew()){
    m_dgainsIn.read();
  }

  if (m_torqueModeIn.isNew())
    m_torqueModeIn.read();

  for (size_t i = 0; i < dof; i++) {
    Pgain[i] = m_pgains.data[i];
    Dgain[i] = m_dgains.data[i];
    
    double q = m_q.data[i];
    double qCom = m_step > 0 ? m_qRef_old[i] + (m_qRef.data[i] - m_qRef_old[i]) / m_step : m_qRef_old[i];

    double dq = (q - m_q_old[i]) / m_dt;
    double dqCom = (qCom - m_qRef_old[i]) / m_dt;

    m_q_old[i] = q;
    m_qRef_old[i] = qCom;

    double tau = m_torqueMode.data[i] ? m_tauRef.data[i] : m_Pgain[i] * (qCom - q) + m_Dgain[i] * (dqCom - dq);

    double tau_limit = m_robot->joint(i)->torqueConst * m_robot->joint(i)->climit * fabs(m_robot->joint(i)->gearRatio);
    
    m_tau.data[i] = std::max(std::min(tau, tau_limit), -tau_limit);

    // if (i == 11 || i == 21)
    //     std::cout << "Rafa, in ModifiedServo::onExecute, for i = " << i << ", q[i] = " << q << ", qRef[i] = " << qRef
    //               << ", tau[i] = " << tau << ", tau_limit[i] = " << tau_limit << ", m_tau[i] = " << m_tau.data[i] << std::endl;
  }

  m_step--;

  m_tau.tm = m_q.tm;
  m_tauOut.write();
  m_pgainsOut.write();
  m_dgainsOut.write();
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t ModifiedServo::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t ModifiedServo::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t ModifiedServo::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t ModifiedServo::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t ModifiedServo::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

void ModifiedServo::readGainFile()
{
  if (m_gain_fname == "") {
    RTC::Properties & prop = getProperties();
    coil::stringTo(m_gain_fname, prop["pdgains_sim_file_name"].c_str());
  }

  m_gain.open(m_gain_fname.c_str());

  if (m_gain.is_open()) {

    double val;

    m_Pgain.resize(m_dof);
    m_Dgain.resize(m_dof);
    
    for (unsigned int i = 0; i < m_dof; i++) {

      if (m_gain >> val)
        m_Pgain[i] = val;
      else
        std::cout << "[" << m_profile.instance_name << "] Gain file [" << m_gain_fname << "] is too short" << std::endl;

      if (m_gain >> val)
        m_Dgain[i] = val;
      else
        std::cout << "[" << m_profile.instance_name << "] Gain file [" << m_gain_fname << "] is too short" << std::endl;
    }

    m_gain.close();

    std::cout << "[" << m_profile.instance_name << "] Gain file [" << m_gain_fname << "] successfully read" << std::endl;
  }
  else
    std::cout << "[" << m_profile.instance_name << "] Gain file [" << m_gain_fname << "] could not be opened" << std::endl;
}

extern "C"
{
 
  void ModifiedServoInit(RTC::Manager* manager)
  {
    coil::Properties profile(modifiedservo_spec);
    manager->registerFactory(profile,
                             RTC::Create<ModifiedServo>,
                             RTC::Delete<ModifiedServo>);
  }
  
};

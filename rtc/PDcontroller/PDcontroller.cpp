// -*- mode: c++; -*-
/*!
 * @file  PDcontroller.cpp
 * @brief Sample PD component
 * $Date$
 *
 * $Id$
 */

#include "PDcontroller.h"
#include <iostream>

// Module specification
// <rtc-template block="module_spec">
static const char* PDcontroller_spec[] =
  {
    "implementation_id", "PDcontroller",
    "type_name",         "PDcontroller",
    "description",       "PDcontroller component",
    "version",           HRPSYS_PACKAGE_VERSION,
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.dt", "0.005",
    "conf.default.t_limit", "1000",
    "conf.default.pdgains_sim_file_name", "",
    ""
  };
// </rtc-template>

PDcontroller::PDcontroller(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_angleIn("angle", m_angle),
    m_angleRefIn("angleRef", m_angleRef),
    m_torqueOut("torque", m_torque),
    dt(0.005),
    // </rtc-template>
    dummy(0)
{
}

PDcontroller::~PDcontroller()
{
}


RTC::ReturnCode_t PDcontroller::onInitialize()
{
  std::cout << m_profile.instance_name << ": onInitialize() " << std::endl;

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("dt", dt, "0.005");
  bindParameter("t_limit", t_limit, "1000");
  bindParameter("pdgains_sim_file_name", gain_fname, "");

  // Set InPort buffers
  addInPort("angle", m_angleIn);
  addInPort("angleRef", m_angleRefIn);
  
  // Set OutPort buffer
  addOutPort("torque", m_torqueOut);
  
  // </rtc-template>

  return RTC::RTC_OK;
}



/*
RTC::ReturnCode_t PDcontroller::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PDcontroller::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PDcontroller::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t PDcontroller::onActivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name << ": on Activated " << std::endl;
  std::cout << "dt: " << dt << std::endl;
  std::cout << "t_limit: " << t_limit << std::endl;
  std::cout << "pdgains_sim_file_name: " << gain_fname << std::endl;

  if(m_angleIn.isNew()){
    m_angleIn.read();
    // initialize length of vectors
    dof = m_angle.data.length();
    qold.resize(dof);
    qold_ref.resize(dof);
    m_torque.data.length(dof);
    m_angleRef.data.length(dof);
    Pgain.resize(dof);
    Dgain.resize(dof);
    gain.open(gain_fname.c_str());
    if (gain.is_open()){
      double tmp;
      for (int i=0; i<dof; i++){
          if (gain >> tmp) {
              Pgain[i] = tmp;
          } else {
              std::cerr << "[" << m_profile.instance_name << "] Gain file [" << gain_fname << "] is too short" << std::endl;
          }
          if (gain >> tmp) {
              Dgain[i] = tmp;
          } else {
              std::cerr << "[" << m_profile.instance_name << "] Gain file [" << gain_fname << "] is too short" << std::endl;
          }
      }
      gain.close();
      std::cerr << "[" << m_profile.instance_name << "] Gain file [" << gain_fname << "] opened" << std::endl;
    }else{
      std::cerr << "[" << m_profile.instance_name << "] Gain file [" << gain_fname << "] not opened" << std::endl;
    }
    // initialize angleRef, old_ref and old with angle
    for(int i=0; i < dof; ++i){
      m_angleRef.data[i] = qold_ref[i] = qold[i] = m_angle.data[i];
    }
  }
  if(m_angleRefIn.isNew()){
    m_angleRefIn.read();
  }

  return RTC::RTC_OK;
}

RTC::ReturnCode_t PDcontroller::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << m_profile.instance_name << ": on Deactivated " << std::endl;
  return RTC::RTC_OK;
}


RTC::ReturnCode_t PDcontroller::onExecute(RTC::UniqueId ec_id)
{
  if(m_angleIn.isNew()){
    m_angleIn.read();
  }
  if(m_angleRefIn.isNew()){
    m_angleRefIn.read();
  }

  for(int i=0; i<dof; i++){
    double q = m_angle.data[i];
    double q_ref = m_angleRef.data[i];
    double dq = (q - qold[i]) / dt;
    double dq_ref = (q_ref - qold_ref[i]) / dt;
    qold[i] = q;
    qold_ref[i] = q_ref;
    m_torque.data[i] = -(q - q_ref) * Pgain[i] - (dq - dq_ref) * Dgain[i];
    if (m_torque.data[i] > t_limit) m_torque.data[i] = t_limit;
    if (m_torque.data[i] < t_limit) m_torque.data[i] = -t_limit;
    // std::cerr << i << " " << m_torque.data[i] << " (" << q << " " << q_ref << ") (" << dq << " " << dq_ref << ") " << Pgain[i] << " " << Dgain[i] << std::endl;
  }
  
  m_torqueOut.write();
  
  return RTC::RTC_OK;
}


/*
  RTC::ReturnCode_t PDcontroller::onAborting(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t PDcontroller::onError(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t PDcontroller::onReset(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t PDcontroller::onStateUpdate(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t PDcontroller::onRateChanged(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

extern "C"
{

  void PDcontrollerInit(RTC::Manager* manager)
  {
    RTC::Properties profile(PDcontroller_spec);
    manager->registerFactory(profile,
                             RTC::Create<PDcontroller>,
                             RTC::Delete<PDcontroller>);
  }

};



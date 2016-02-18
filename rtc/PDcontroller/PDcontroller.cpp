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
#include <coil/stringutil.h>

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
    "conf.default.pdgains_sim_file_name", "",
    "conf.default.debugLevel", "0",
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
    dummy(0),
    gain_fname(""),
    dof(0), loop(0)
{
}

PDcontroller::~PDcontroller()
{
}


RTC::ReturnCode_t PDcontroller::onInitialize()
{
  std::cout << m_profile.instance_name << ": onInitialize() " << std::endl;

  RTC::Properties& prop = getProperties();
  coil::stringTo(dt, prop["dt"].c_str());
  ref_dt = dt;
  coil::stringTo(ref_dt, prop["ref_dt"].c_str());
  nstep = ref_dt/dt;
  step = nstep;

  m_robot = hrp::BodyPtr(new hrp::Body());

  RTC::Manager& rtcManager = RTC::Manager::instance();
  std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
  int comPos = nameServer.find(",");
  if (comPos < 0){
      comPos = nameServer.length();
  }
  nameServer = nameServer.substr(0, comPos);
  RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());
  if (!loadBodyFromModelLoader(m_robot, prop["model"].c_str(), 
                               CosNaming::NamingContext::_duplicate(naming.getRootContext())
                               )){
      std::cerr << "[" << m_profile.instance_name << "] failed to load model[" << prop["model"] << "]" 
                << std::endl;
  }

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("pdgains_sim_file_name", gain_fname, "");
  bindParameter("debugLevel", m_debugLevel, "0");

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
  if(m_angleIn.isNew()){
    m_angleIn.read();
    if (dof == 0) {
        dof = m_angle.data.length();
        readGainFile();
    }
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
  loop++;
  if(m_angleIn.isNew()){
    m_angleIn.read();
    if (dof == 0) {
        dof = m_angle.data.length();
        readGainFile();
    }
  }
  if(m_angleRefIn.isNew()){
    m_angleRefIn.read();
    step = nstep;
  }

  for(int i=0; i<dof; i++){
    double q = m_angle.data[i];
    double q_ref = step > 0 ? qold_ref[i] + (m_angleRef.data[i] - qold_ref[i])/step : qold_ref[i];
    double dq = (q - qold[i]) / dt;
    double dq_ref = (q_ref - qold_ref[i]) / dt;
    qold[i] = q;
    qold_ref[i] = q_ref;
    m_torque.data[i] = -(q - q_ref) * Pgain[i] - (dq - dq_ref) * Dgain[i];
    double tlimit;
    if (m_robot && m_robot->numJoints() == dof) {
        tlimit = m_robot->joint(i)->climit * m_robot->joint(i)->gearRatio * m_robot->joint(i)->torqueConst * tlimit_ratio[i];
    } else {
        tlimit = (std::numeric_limits<double>::max)() * tlimit_ratio[i];
        if (i == 0 && loop % 500 == 0) {
            std::cerr << "[" << m_profile.instance_name << "] m_robot is not set properly!! Maybe ModelLoader is missing?" << std::endl;
        }
    }
    if (loop % 100 == 0 && m_debugLevel == 1) {
        std::cerr << "[" << m_profile.instance_name << "] joint = "
                  << i << ", tq = " << m_torque.data[i] << ", q,qref = (" << q << ", " << q_ref << "), dq,dqref = (" << dq << ", " << dq_ref << "), pd = (" << Pgain[i] << ", " << Dgain[i] << "), tlimit = " << tlimit << std::endl;
    }
    m_torque.data[i] = std::max(std::min(m_torque.data[i], tlimit), -tlimit);
  }
  step--; 
  
  m_torqueOut.write();
  
  return RTC::RTC_OK;
}

void PDcontroller::readGainFile()
{
    if (gain_fname == "") {
        RTC::Properties& prop = getProperties();
        coil::stringTo(gain_fname, prop["pdgains_sim_file_name"].c_str());
    }
    // initialize length of vectors
    qold.resize(dof);
    qold_ref.resize(dof);
    m_torque.data.length(dof);
    m_angleRef.data.length(dof);
    Pgain.resize(dof);
    Dgain.resize(dof);
    gain.open(gain_fname.c_str());
    tlimit_ratio.resize(dof);
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
    // tlimit_ratio initialize
    {
        RTC::Properties& prop = getProperties();
        if (prop["pdcontrol_tlimit_ratio"] != "") {
            coil::vstring tlimit_ratio_str = coil::split(prop["pdcontrol_tlimit_ratio"], ",");
            if (tlimit_ratio_str.size() == dof) {
                for (size_t i = 0; i < dof; i++) {
                    coil::stringTo(tlimit_ratio[i], tlimit_ratio_str[i].c_str());
                }
                std::cerr << "[" << m_profile.instance_name << "] tlimit_ratio is set to " << prop["pdcontrol_tlimit_ratio"] << std::endl;
            } else {
                for (size_t i = 0; i < dof; i++) {
                    tlimit_ratio[i] = 1.0;
                }
                std::cerr << "[" << m_profile.instance_name << "] pdcontrol_tlimit_ratio found, but invalid length (" << tlimit_ratio_str.size() << " != " << dof << ")." << std::endl;
                std::cerr << "[" << m_profile.instance_name << "] All tlimit_ratio are set to 1.0." << std::endl;
            }
        } else {
            for (size_t i = 0; i < dof; i++) {
                tlimit_ratio[i] = 1.0;
            }
            std::cerr << "[" << m_profile.instance_name << "] No pdcontrol_tlimit_ratio found." << std::endl;
            std::cerr << "[" << m_profile.instance_name << "] All tlimit_ratio are set to 1.0." << std::endl;
        }
    }
    // initialize angleRef, old_ref and old with angle
    for(int i=0; i < dof; ++i){
      m_angleRef.data[i] = qold_ref[i] = qold[i] = m_angle.data[i];
    }
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



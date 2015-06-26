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
#include <rtm/CorbaNaming.h>
#include <hrpModel/ModelLoaderUtil.h>
#include <math.h>
#include <hrpModel/Link.h>
#include <hrpModel/Sensor.h>

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
    "conf.default.debugLevel", "0",
    ""
};
// </rtc-template>

EmergencyStopper::EmergencyStopper(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      // <rtc-template block="initializer">
      m_qRefIn("qRef", m_qRef),
      m_emergencySignalIn("emergencySignal", m_emergencySignal),
      m_qOut("q", m_q),
      m_EmergencyStopperServicePort("EmergencyStopperService"),
      // </rtc-template>
      m_robot(hrp::BodyPtr()),
      m_debugLevel(0),
      dummy(0),
      loop(0)
{
    m_service0.emergencystopper(this);
}

EmergencyStopper::~EmergencyStopper()
{
}


#define DEBUGP ((m_debugLevel==1 && loop%200==0) || m_debugLevel > 1 )
RTC::ReturnCode_t EmergencyStopper::onInitialize()
{
    std::cerr << "[" << m_profile.instance_name << "] onInitialize()" << std::endl;
    // <rtc-template block="bind_config">
    // Bind variables and configuration variable
    bindParameter("debugLevel", m_debugLevel, "0");

    // Registration: InPort/OutPort/Service
    // <rtc-template block="registration">
    // Set InPort buffers
    addInPort("qRef", m_qRefIn);
    addInPort("emergencySignal", m_emergencySignalIn);

    // Set OutPort buffer
    addOutPort("q", m_qOut);

    // Set service provider to Ports
    m_EmergencyStopperServicePort.registerProvider("service0", "EmergencyStopperService", m_service0);

    // Set service consumers to Ports

    // Set CORBA Service Ports
    addPort(m_EmergencyStopperServicePort);

    // </rtc-template>

    // Setup robot model
    RTC::Properties& prop = getProperties();
    coil::stringTo(m_dt, prop["dt"].c_str());
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
        std::cerr << "[" << m_profile.instance_name << "] failed to load model[" << prop["model"] << "]" << std::endl;
    }

    is_stop_mode = false;
    is_initialized = false;

    recover_time = retrieve_time = 0;
    recover_time_dt = 1.0;
    default_recover_time = 2.5/m_dt;
    default_retrieve_time = 1;
    m_stop_posture = new double[m_robot->numJoints()];
    m_interpolator = new interpolator(m_robot->numJoints(), recover_time_dt);

    m_q.data.length(m_robot->numJoints());
    for(int i=0; i<m_robot->numJoints(); i++){
        m_q.data[i] = 0;
        m_stop_posture[i] = 0;
    }

    return RTC::RTC_OK;
}




RTC::ReturnCode_t EmergencyStopper::onFinalize()
{
    delete m_interpolator;
    delete m_stop_posture;
    return RTC::RTC_OK;
}

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
    int numJoints = m_robot->numJoints();
    loop++;

    if (!is_initialized) {
        if (m_qRefIn.isNew()) {
            m_qRefIn.read();
            is_initialized = true;
        } else {
            return RTC::RTC_OK;
        }
    }

    if (m_qRefIn.isNew()) {
        m_qRefIn.read();
        assert(m_qRef.data.length() == numJoints);
        if (!is_stop_mode) {
            std::vector<double> current_posture;
            for ( int i = 0; i < m_qRef.data.length(); i++ ) {
                current_posture.push_back(m_qRef.data[i]);
            }
            m_input_posture_queue.push(current_posture);
            if (m_input_posture_queue.size() > default_retrieve_time) {
                m_input_posture_queue.pop();
            }
            for ( int i = 0; i < m_qRef.data.length(); i++ ) {
                m_stop_posture[i] = m_input_posture_queue.front()[i];
            }
        } else {
            while (!m_input_posture_queue.empty()) m_input_posture_queue.pop();
        }
    }

    if (m_emergencySignalIn.isNew()){
        m_emergencySignalIn.read();
        if (!is_stop_mode) {
            std::cerr << "[" << m_profile.instance_name << "] emergencySignal is set!" << std::endl;
            is_stop_mode = true;
        }
    }

    if (DEBUGP) {
        std::cerr << "[" << m_profile.instance_name << "] is_stop_mode : " << is_stop_mode << " recover_time : "  << recover_time << "[s] retrieve_time : " << retrieve_time << "[s]" << std::endl;
    }

    //     mode : is_stop_mode : recover_time  : set as q
    // release  :        false :            0  : qRef
    // recover  :        false :         >  0  : q'
    // stop     :         true :  do not care  : q(do nothing)
    if (!is_stop_mode) {
        if (recover_time > 0) {
            recover_time = recover_time - recover_time_dt;
            m_interpolator->setGoal(m_qRef.data.get_buffer(), recover_time);
            m_interpolator->get(m_q.data.get_buffer());
        } else {
            for ( int i = 0; i < m_q.data.length(); i++ ) {
                m_q.data[i] = m_qRef.data[i];
            }
        }
    } else { // stop mode
        recover_time = default_recover_time;
        if (retrieve_time > 0 ) {
            retrieve_time = retrieve_time - recover_time_dt;
            m_interpolator->setGoal(m_stop_posture, retrieve_time);
            m_interpolator->get(m_q.data.get_buffer());
        } else {
            // Do nothing
        }
    }

    if (DEBUGP) {
        std::cerr << "q: ";
        for (int i = 0; i < numJoints; i++) {
            std::cerr << " " << m_q.data[i] ;
        }
        std::cerr << std::endl;
    }
    m_qOut.write();
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

bool EmergencyStopper::stopMotion()
{
    if (!is_stop_mode) {
        is_stop_mode = true;
        m_interpolator->set(m_qRef.data.get_buffer());
        retrieve_time = default_retrieve_time;
        std::cerr << "[" << m_profile.instance_name << "] stopMotion is called" << std::endl;
    }
    return true;
}

bool EmergencyStopper::releaseMotion()
{
    if (is_stop_mode) {
        is_stop_mode = false;
        std::cerr << "[" << m_profile.instance_name << "] releaseMotion is called" << std::endl;
    }
    return true;
}


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



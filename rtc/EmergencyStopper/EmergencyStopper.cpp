// -*- C++ -*-
/*!
 * @file  EmergencyStopper.cpp
 * @brief emergency stopper
 * $Date$
 *
 * $Id$
 */

#include "util/VectorConvert.h"
#include <rtm/CorbaNaming.h>
#include <hrpModel/ModelLoaderUtil.h>
#include <math.h>
#include <hrpModel/Link.h>
#include <hrpModel/Sensor.h>
#include "RobotHardwareService.hh"

#include "EmergencyStopper.h"
#include "../SoftErrorLimiter/beep.h"

typedef coil::Guard<coil::Mutex> Guard;

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
      m_emergencyModeOut("emergencyMode", m_emergencyMode),
      m_EmergencyStopperServicePort("EmergencyStopperService"),
      m_servoStateIn("servoStateIn", m_servoState),
      // </rtc-template>
      m_robot(hrp::BodyPtr()),
      m_debugLevel(0),
      dummy(0),
      loop(0),
      emergency_stopper_beep_count(0)
{
    m_service0.emergencystopper(this);
    init_beep();
    start_beep(3136);
}

EmergencyStopper::~EmergencyStopper()
{
    quit_beep();
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
    addInPort("servoStateIn", m_servoStateIn);

    // Set OutPort buffer
    addOutPort("q", m_qOut);
    addOutPort("emergencyMode", m_emergencyModeOut);

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
    OpenHRP::BodyInfo_var binfo;
    binfo = hrp::loadBodyInfo(prop["model"].c_str(),
                              CosNaming::NamingContext::_duplicate(naming.getRootContext()));
    if (CORBA::is_nil(binfo)) {
        std::cerr << "failed to load model[" << prop["model"] << "]"
                  << std::endl;
        return RTC::RTC_ERROR;
    }
    if (!loadBodyFromBodyInfo(m_robot, binfo)) {
        std::cerr << "failed to load model[" << prop["model"] << "] in "
                  << m_profile.instance_name << std::endl;
        return RTC::RTC_ERROR;
    }

    // Setting for wrench data ports (real + virtual)
    OpenHRP::LinkInfoSequence_var lis = binfo->links();
    std::vector<std::string> fsensor_names;
    //   find names for real force sensors
    for ( int k = 0; k < lis->length(); k++ ) {
        OpenHRP::SensorInfoSequence& sensors = lis[k].sensors;
        for ( int l = 0; l < sensors.length(); l++ ) {
            if ( std::string(sensors[l].type) == "Force" ) {
                fsensor_names.push_back(std::string(sensors[l].name));
            }
        }
    }
    int npforce = fsensor_names.size();
    //   find names for virtual force sensors
    coil::vstring virtual_force_sensor = coil::split(prop["virtual_force_sensor"], ",");
    int nvforce = virtual_force_sensor.size()/10;
    for (unsigned int i=0; i<nvforce; i++){
        fsensor_names.push_back(virtual_force_sensor[i*10+0]);
    }
    //   add ports for all force sensors
    int nforce  = npforce + nvforce;
    m_wrenchesRef.resize(nforce);
    m_wrenches.resize(nforce);
    m_wrenchesIn.resize(nforce);
    m_wrenchesOut.resize(nforce);
    for (unsigned int i=0; i<nforce; i++){
        m_wrenchesIn[i] = new InPort<TimedDoubleSeq>(std::string(fsensor_names[i]+"In").c_str(), m_wrenchesRef[i]);
        m_wrenchesOut[i] = new OutPort<TimedDoubleSeq>(std::string(fsensor_names[i]+"Out").c_str(), m_wrenches[i]);
        m_wrenchesRef[i].data.length(6);
        m_wrenchesRef[i].data[0] = m_wrenchesRef[i].data[1] = m_wrenchesRef[i].data[2] = 0.0;
        m_wrenchesRef[i].data[3] = m_wrenchesRef[i].data[4] = m_wrenchesRef[i].data[5] = 0.0;
        m_wrenches[i].data.length(6);
        m_wrenches[i].data[0] = m_wrenches[i].data[1] = m_wrenches[i].data[2] = 0.0;
        m_wrenches[i].data[3] = m_wrenches[i].data[4] = m_wrenches[i].data[5] = 0.0;
        registerInPort(std::string(fsensor_names[i]+"In").c_str(), *m_wrenchesIn[i]);
        registerOutPort(std::string(fsensor_names[i]+"Out").c_str(), *m_wrenchesOut[i]);
    }

    // initialize member variables
    is_stop_mode = prev_is_stop_mode = false;
    is_initialized = false;

    recover_time = retrieve_time = 0;
    recover_time_dt = 1.0;
    default_recover_time = 2.5/m_dt;
    default_retrieve_time = 1;
    //default_retrieve_time = 1.0/m_dt;
    m_stop_posture = new double[m_robot->numJoints()];
    m_stop_wrenches = new double[nforce*6];
    m_tmp_wrenches = new double[nforce*6];
    m_interpolator = new interpolator(m_robot->numJoints(), recover_time_dt);
    m_interpolator->setName(std::string(m_profile.instance_name)+" interpolator");
    m_wrenches_interpolator = new interpolator(nforce*6, recover_time_dt);
    m_wrenches_interpolator->setName(std::string(m_profile.instance_name)+" interpolator wrenches");

    m_q.data.length(m_robot->numJoints());
    for(int i=0; i<m_robot->numJoints(); i++){
        m_q.data[i] = 0;
        m_stop_posture[i] = 0;
    }
    for(int i=0; i<nforce; i++){
        for(int j=0; j<6; j++){
            m_wrenches[i].data[j] = 0;
            m_stop_wrenches[i*6+j] = 0;
        }
    }

    m_servoState.data.length(m_robot->numJoints());
    for(int i = 0; i < m_robot->numJoints(); i++) {
        m_servoState.data[i].length(1);
        int status = 0;
        status |= 1<< OpenHRP::RobotHardwareService::CALIB_STATE_SHIFT;
        status |= 1<< OpenHRP::RobotHardwareService::POWER_STATE_SHIFT;
        status |= 1<< OpenHRP::RobotHardwareService::SERVO_STATE_SHIFT;
        status |= 0<< OpenHRP::RobotHardwareService::SERVO_ALARM_SHIFT;
        status |= 0<< OpenHRP::RobotHardwareService::DRIVER_TEMP_SHIFT;
        m_servoState.data[i][0] = status;
    }

    emergency_stopper_beep_freq = static_cast<int>(1.0/(2.0*m_dt)); // 2 times / 1[s]
    return RTC::RTC_OK;
}




RTC::ReturnCode_t EmergencyStopper::onFinalize()
{
    delete m_interpolator;
    delete m_wrenches_interpolator;
    delete m_stop_posture;
    delete m_stop_wrenches;
    delete m_tmp_wrenches;
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
    Guard guard(m_mutex);
    if (is_stop_mode) {
        is_stop_mode = false;
        recover_time = 0;
        m_interpolator->setGoal(m_qRef.data.get_buffer(), m_dt);
        m_interpolator->get(m_q.data.get_buffer());
    }
    return RTC::RTC_OK;
}

RTC::ReturnCode_t EmergencyStopper::onExecute(RTC::UniqueId ec_id)
{
    int numJoints = m_robot->numJoints();
    loop++;
    if (m_servoStateIn.isNew()) {
        m_servoStateIn.read();
    }
    if (!is_initialized) {
        if (m_qRefIn.isNew()) {
            m_qRefIn.read();
            is_initialized = true;
        } else {
            return RTC::RTC_OK;
        }
    }

    // read data
    if (m_qRefIn.isNew()) {
        // joint angle
        m_qRefIn.read();
        assert(m_qRef.data.length() == numJoints);
        std::vector<double> current_posture;
        for ( int i = 0; i < m_qRef.data.length(); i++ ) {
            current_posture.push_back(m_qRef.data[i]);
        }
        m_input_posture_queue.push(current_posture);
        while (m_input_posture_queue.size() > default_retrieve_time) {
            m_input_posture_queue.pop();
        }
        if (!is_stop_mode) {
            for ( int i = 0; i < m_qRef.data.length(); i++ ) {
                if (recover_time > 0) { // Until releasing is finished, do not use m_stop_posture in input queue because too large error.
                    m_stop_posture[i] = m_q.data[i];
                } else {
                    m_stop_posture[i] = m_input_posture_queue.front()[i];
                }
            }
        }
        // wrench
        for ( size_t i = 0; i < m_wrenchesIn.size(); i++ ) {
            if ( m_wrenchesIn[i]->isNew() ) {
                m_wrenchesIn[i]->read();
            }
        }
        std::vector<double> current_wrench;
        for ( int i= 0; i < m_wrenchesRef.size(); i++ ) {
            for (int j = 0; j < 6; j++ ) {
                current_wrench.push_back(m_wrenchesRef[i].data[j]);
            }
        }
        m_input_wrenches_queue.push(current_wrench);
        while (m_input_wrenches_queue.size() > default_retrieve_time) {
            m_input_wrenches_queue.pop();
        }
        if (!is_stop_mode) {
            for ( int i= 0; i < m_wrenchesRef.size(); i++ ) {
                for (int j = 0; j < 6; j++ ) {
                    if (recover_time > 0) {
                        m_stop_wrenches[i*6+j] = m_wrenches[i].data[j];
                    } else {
                        m_stop_wrenches[i*6+j] = m_input_wrenches_queue.front()[i*6+j];
                    }
                }
            }
        }
    }

    if (m_emergencySignalIn.isNew()){
        m_emergencySignalIn.read();
        if ( m_emergencySignal.data == 0 ) {
            Guard guard(m_mutex);
            std::cerr << "[" << m_profile.instance_name << "] emergencySignal is reset!" << std::endl;
            is_stop_mode = false;
        } else if (!is_stop_mode) {
            Guard guard(m_mutex);
            std::cerr << "[" << m_profile.instance_name << "] emergencySignal is set!" << std::endl;
            is_stop_mode = true;
        }
    }
    if (is_stop_mode && !prev_is_stop_mode) {
        retrieve_time = default_retrieve_time;
        // Reflect current output joint angles to interpolator state
        m_interpolator->set(m_q.data.get_buffer());
        get_wrenches_array_from_data(m_wrenches, m_tmp_wrenches);
        m_wrenches_interpolator->set(m_tmp_wrenches);
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
            get_wrenches_array_from_data(m_wrenchesRef, m_tmp_wrenches);
            m_wrenches_interpolator->setGoal(m_tmp_wrenches, recover_time);
            m_wrenches_interpolator->get(m_tmp_wrenches);
            set_wrenches_data_from_array(m_wrenches, m_tmp_wrenches);
        } else {
            for ( int i = 0; i < m_q.data.length(); i++ ) {
                m_q.data[i] = m_qRef.data[i];
            }
            for ( int i = 0; i < m_wrenches.size(); i++ ) {
                for ( int j = 0; j < 6; j++ ) {
                    m_wrenches[i].data[j] = m_wrenchesRef[i].data[j];
                }
            }
        }
    } else { // stop mode
        recover_time = default_recover_time;
        if (retrieve_time > 0 ) {
            retrieve_time = retrieve_time - recover_time_dt;
            m_interpolator->setGoal(m_stop_posture, retrieve_time);
            m_interpolator->get(m_q.data.get_buffer());
            m_wrenches_interpolator->setGoal(m_stop_wrenches, retrieve_time);
            m_wrenches_interpolator->get(m_tmp_wrenches);
            set_wrenches_data_from_array(m_wrenches, m_tmp_wrenches);
        } else {
            // Do nothing
        }
    }

    // write data
    if (DEBUGP) {
        std::cerr << "q: ";
        for (int i = 0; i < numJoints; i++) {
            std::cerr << " " << m_q.data[i] ;
        }
        std::cerr << std::endl;
        std::cerr << "wrenches: ";
        for (int i = 0; i < m_wrenches.size(); i++) {
            for (int j = 0; j < 6; j++ ) {
                std::cerr << " " << m_wrenches[i].data[j];
            }
        }
        std::cerr << std::endl;
    }
    m_q.tm = m_qRef.tm;
    m_qOut.write();
    for (size_t i = 0; i < m_wrenches.size(); i++) {
      m_wrenches[i].tm = m_qRef.tm;
    }
    for (size_t i = 0; i < m_wrenchesOut.size(); i++) {
      m_wrenchesOut[i]->write();
    }

    m_emergencyMode.data = is_stop_mode;
    m_emergencyMode.tm = m_qRef.tm;
    m_emergencyModeOut.write();

    prev_is_stop_mode = is_stop_mode;

    // beep sound for emergency stop alert
    //  check servo for emergency stop beep sound
    bool has_servoOn = false;
    for (int i = 0; i < m_robot->numJoints(); i++ ){
        int servo_state = (m_servoState.data[i][0] & OpenHRP::RobotHardwareService::SERVO_STATE_MASK) >> OpenHRP::RobotHardwareService::SERVO_STATE_SHIFT;
        has_servoOn = has_servoOn || (servo_state == 1);
    }
    //  beep
    if ( is_stop_mode && has_servoOn ) { // If stop mode and some joint is servoOn
        if ( emergency_stopper_beep_count % emergency_stopper_beep_freq == 0 && emergency_stopper_beep_count % (emergency_stopper_beep_freq * 3) != 0 ) start_beep(2352, emergency_stopper_beep_freq*0.7);
        else stop_beep();
        emergency_stopper_beep_count++;
    } else {
        emergency_stopper_beep_count = 0;
    }
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
    Guard guard(m_mutex);
    if (!is_stop_mode) {
        is_stop_mode = true;
        std::cerr << "[" << m_profile.instance_name << "] stopMotion is called" << std::endl;
    }
    return true;
}

bool EmergencyStopper::releaseMotion()
{
    Guard guard(m_mutex);
    if (is_stop_mode) {
        is_stop_mode = false;
        std::cerr << "[" << m_profile.instance_name << "] releaseMotion is called" << std::endl;
    }
    return true;
}

bool EmergencyStopper::getEmergencyStopperParam(OpenHRP::EmergencyStopperService::EmergencyStopperParam& i_param)
{
    std::cerr << "[" << m_profile.instance_name << "] getEmergencyStopperParam" << std::endl;
    i_param.default_recover_time = default_recover_time*m_dt;
    i_param.default_retrieve_time = default_retrieve_time*m_dt;
    i_param.is_stop_mode = is_stop_mode;
    return true;
};

bool EmergencyStopper::setEmergencyStopperParam(const OpenHRP::EmergencyStopperService::EmergencyStopperParam& i_param)
{
    std::cerr << "[" << m_profile.instance_name << "] setEmergencyStopperParam" << std::endl;
    default_recover_time = i_param.default_recover_time/m_dt;
    default_retrieve_time = i_param.default_retrieve_time/m_dt;
    std::cerr << "[" << m_profile.instance_name << "]   default_recover_time = " << default_recover_time*m_dt << "[s], default_retrieve_time = " << default_retrieve_time*m_dt << "[s]" << std::endl;
    return true;
};

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



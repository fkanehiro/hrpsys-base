// -*- C++ -*-
/*!
 * @file  SequencePlayer.cpp
 * @brief sequence player component
 * $Date$
 *
 * $Id$
 */

#include <rtm/CorbaNaming.h>
#include <hrpModel/Link.h>
#include <hrpModel/ModelLoaderUtil.h>
#include "SequencePlayer.h"
#include "util/VectorConvert.h"
#include <hrpModel/JointPath.h>
#include <hrpUtil/MatrixSolvers.h>
#include "../ImpedanceController/JointPathEx.h"

typedef coil::Guard<coil::Mutex> Guard;

// Module specification
// <rtc-template block="module_spec">
static const char* sequenceplayer_spec[] =
    {
        "implementation_id", "SequencePlayer",
        "type_name",         "SequencePlayer",
        "description",       "sequence player component",
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

SequencePlayer::SequencePlayer(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      // <rtc-template block="initializer">
      m_qInitIn("qInit", m_qInit),
      m_basePosInitIn("basePosInit", m_basePosInit),
      m_baseRpyInitIn("baseRpyInit", m_baseRpyInit),
      m_zmpRefInitIn("zmpRefInit", m_zmpRefInit),
      m_qRefOut("qRef", m_qRef),
      m_tqRefOut("tqRef", m_tqRef),
      m_zmpRefOut("zmpRef", m_zmpRef),
      m_accRefOut("accRef", m_accRef),
      m_basePosOut("basePos", m_basePos),
      m_baseRpyOut("baseRpy", m_baseRpy),
      m_optionalDataOut("optionalData", m_optionalData),
      m_SequencePlayerServicePort("SequencePlayerService"),
      // </rtc-template>
      m_waitSem(0),
      m_robot(hrp::BodyPtr()),
      m_debugLevel(0),
      m_error_pos(0.0001),
      m_error_rot(0.001),
      m_iteration(50),
      dummy(0)
{
    m_service0.player(this);
    m_clearFlag = false;
    m_waitFlag = false;
}

SequencePlayer::~SequencePlayer()
{
}


RTC::ReturnCode_t SequencePlayer::onInitialize()
{
    std::cout << "SequencePlayer::onInitialize()" << std::endl;
    // Registration: InPort/OutPort/Service
    // <rtc-template block="registration">
    // Set InPort buffers
    addInPort("qInit", m_qInitIn);
    addInPort("basePosInit", m_basePosInitIn);
    addInPort("baseRpyInit", m_baseRpyInitIn);
    addInPort("zmpRefInit", m_zmpRefInitIn);
  
    // Set OutPort buffer
    addOutPort("qRef", m_qRefOut);
    addOutPort("tqRef", m_tqRefOut);
    addOutPort("zmpRef", m_zmpRefOut);
    addOutPort("accRef", m_accRefOut);
    addOutPort("basePos", m_basePosOut);
    addOutPort("baseRpy", m_baseRpyOut);
    addOutPort("optionalData", m_optionalDataOut);
  
    // Set service provider to Ports
    m_SequencePlayerServicePort.registerProvider("service0", "SequencePlayerService", m_service0);
  
    // Set service consumers to Ports
  
    // Set CORBA Service Ports
    addPort(m_SequencePlayerServicePort);
  
    // </rtc-template>
    // <rtc-template block="bind_config">
    // Bind variables and configuration variable
  
    bindParameter("debugLevel", m_debugLevel, "0");
    // </rtc-template>

    RTC::Properties& prop = getProperties();
    coil::stringTo(dt, prop["dt"].c_str());

    m_robot = hrp::BodyPtr(new Body());

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
        std::cerr << "failed to load model[" << prop["model"] << "]" 
                  << std::endl;
    }

    unsigned int dof = m_robot->numJoints();


    // Setting for wrench data ports (real + virtual)
    std::vector<std::string> fsensor_names;
    //   find names for real force sensors
    int npforce = m_robot->numSensors(hrp::Sensor::FORCE);
    for (unsigned int i=0; i<npforce; i++){
      fsensor_names.push_back(m_robot->sensor(hrp::Sensor::FORCE, i)->name);
    }
    //   find names for virtual force sensors
    coil::vstring virtual_force_sensor = coil::split(prop["virtual_force_sensor"], ",");
    int nvforce = virtual_force_sensor.size()/10;
    for (unsigned int i=0; i<nvforce; i++){
      fsensor_names.push_back(virtual_force_sensor[i*10+0]);
    }
    //   add ports for all force sensors
    int nforce  = npforce + nvforce;
    m_wrenches.resize(nforce);
    m_wrenchesOut.resize(nforce);
    for (unsigned int i=0; i<nforce; i++){
      m_wrenchesOut[i] = new OutPort<TimedDoubleSeq>(std::string(fsensor_names[i]+"Ref").c_str(), m_wrenches[i]);
      m_wrenches[i].data.length(6);
      registerOutPort(std::string(fsensor_names[i]+"Ref").c_str(), *m_wrenchesOut[i]);
    }

    if (prop.hasKey("seq_optional_data_dim")) {
      coil::stringTo(optional_data_dim, prop["seq_optional_data_dim"].c_str());
    } else {
      optional_data_dim = 1;
    }

    m_seq = new seqplay(dof, dt, nforce, optional_data_dim);

    m_qInit.data.length(dof);
    for (unsigned int i=0; i<dof; i++) m_qInit.data[i] = 0.0;
    Link *root = m_robot->rootLink();
    m_basePosInit.data.x = root->p[0]; m_basePosInit.data.y = root->p[1]; m_basePosInit.data.z = root->p[2];
    hrp::Vector3 rpy = hrp::rpyFromRot(root->R);
    m_baseRpyInit.data.r = rpy[0]; m_baseRpyInit.data.p = rpy[1]; m_baseRpyInit.data.y = rpy[2];
    m_zmpRefInit.data.x = 0; m_zmpRefInit.data.y = 0; m_zmpRefInit.data.z = 0;

    // allocate memory for outPorts
    m_qRef.data.length(dof);
    m_tqRef.data.length(dof);
    m_optionalData.data.length(optional_data_dim);

    return RTC::RTC_OK;
}



RTC::ReturnCode_t SequencePlayer::onFinalize()
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    return RTC::RTC_OK;
}

/*
  RTC::ReturnCode_t SequencePlayer::onStartup(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t SequencePlayer::onShutdown(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

RTC::ReturnCode_t SequencePlayer::onActivated(RTC::UniqueId ec_id)
{
    std::cout << "SequencePlayer::onActivated(" << ec_id << ")" << std::endl;
    
    return RTC::RTC_OK;
}

/*
  RTC::ReturnCode_t SequencePlayer::onDeactivated(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

RTC::ReturnCode_t SequencePlayer::onExecute(RTC::UniqueId ec_id)
{
    static int loop = 0;
    loop++;
    if ( m_debugLevel > 0 && loop % 1000 == 0) {
        std::cerr << __PRETTY_FUNCTION__ << "(" << ec_id << ")" << std::endl;
    }
    if (m_qInitIn.isNew()) m_qInitIn.read();
    if (m_basePosInitIn.isNew()) m_basePosInitIn.read();
    if (m_baseRpyInitIn.isNew()) m_baseRpyInitIn.read();
    if (m_zmpRefInitIn.isNew()) m_zmpRefInitIn.read();

    if (m_gname != "" && m_seq->isEmpty(m_gname.c_str())){
        if (m_waitFlag){
            m_gname = "";
            m_waitFlag = false;
            m_waitSem.post();
        }
    }
    if (m_seq->isEmpty()){
        m_clearFlag = false;
        if (m_waitFlag){
            m_waitFlag = false;
            m_waitSem.post();
        }
    }else{
	Guard guard(m_mutex);

        double zmp[3], acc[3], pos[3], rpy[3], wrenches[6*m_wrenches.size()];
        m_seq->get(m_qRef.data.get_buffer(), zmp, acc, pos, rpy, m_tqRef.data.get_buffer(), wrenches, m_optionalData.data.get_buffer());
        m_zmpRef.data.x = zmp[0];
        m_zmpRef.data.y = zmp[1];
        m_zmpRef.data.z = zmp[2];
        m_accRef.data.ax = acc[0]; 
        m_accRef.data.ay = acc[1]; 
        m_accRef.data.az = acc[2]; 
        m_basePos.data.x = pos[0];
        m_basePos.data.y = pos[1];
        m_basePos.data.z = pos[2];
        m_baseRpy.data.r = rpy[0];
        m_baseRpy.data.p = rpy[1];
        m_baseRpy.data.y = rpy[2];
        size_t force_i = 0;
        for (size_t i = 0; i < m_wrenches.size(); i++) {
          m_wrenches[i].data[0] = wrenches[force_i++];
          m_wrenches[i].data[1] = wrenches[force_i++];
          m_wrenches[i].data[2] = wrenches[force_i++];
          m_wrenches[i].data[3] = wrenches[force_i++];
          m_wrenches[i].data[4] = wrenches[force_i++];
          m_wrenches[i].data[5] = wrenches[force_i++];
        }
        m_qRef.tm = m_qInit.tm;
        m_qRefOut.write();
        m_tqRefOut.write();
        m_zmpRefOut.write();
        m_accRefOut.write();
        m_basePosOut.write();
        m_baseRpyOut.write();
        m_optionalDataOut.write();
        for (size_t i = 0; i < m_wrenchesOut.size(); i++) {
          m_wrenchesOut[i]->write();
        }

        if (m_clearFlag){
            m_seq->clear(0.001);
        }
    }
    return RTC::RTC_OK;
}

/*
  RTC::ReturnCode_t SequencePlayer::onAborting(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t SequencePlayer::onError(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t SequencePlayer::onReset(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t SequencePlayer::onStateUpdate(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t SequencePlayer::onRateChanged(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

void SequencePlayer::setClearFlag()
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    m_clearFlag = true;
}

void SequencePlayer::waitInterpolation()
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    m_waitFlag = true;
    m_waitSem.wait();
}

bool SequencePlayer::waitInterpolationOfGroup(const char *gname)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    m_gname = gname;
    m_waitFlag = true;
    m_waitSem.wait();
    return true;
}


bool SequencePlayer::setJointAngle(short id, double angle, double tm)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);
    if (!setInitialState()) return false;
    dvector q(m_robot->numJoints());
    m_seq->getJointAngles(q.data());
    q[id] = angle;
    for (int i=0; i<m_robot->numJoints(); i++){
        hrp::Link *j = m_robot->joint(i);
        if (j) j->q = q[i];
    }
    m_robot->calcForwardKinematics();
    hrp::Vector3 absZmp = m_robot->calcCM();
    absZmp[2] = 0;
    hrp::Link *root = m_robot->rootLink();
    hrp::Vector3 relZmp = root->R.transpose()*(absZmp - root->p);
    m_seq->setJointAngles(q.data(), tm);
    m_seq->setZmp(relZmp.data(), tm);
    return true;
}

bool SequencePlayer::setJointAngles(const double *angles, double tm)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);
    if (!setInitialState()) return false;
    for (int i=0; i<m_robot->numJoints(); i++){
        hrp::Link *j = m_robot->joint(i);
        if (j) j->q = angles[i];
    }
    m_robot->calcForwardKinematics();
    hrp::Vector3 absZmp = m_robot->calcCM();
    absZmp[2] = 0;
    hrp::Link *root = m_robot->rootLink();
    hrp::Vector3 relZmp = root->R.transpose()*(absZmp - root->p);
    std::vector<const double*> v_poss;
    std::vector<double> v_tms;
    v_poss.push_back(angles);
    v_tms.push_back(tm);
    m_seq->setJointAnglesSequence(v_poss, v_tms);
    m_seq->setZmp(relZmp.data(), tm);
    return true;
}

bool SequencePlayer::setJointAngles(const double *angles, const bool *mask, 
                                    double tm)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);

    if (!setInitialState()) return false;

    double pose[m_robot->numJoints()];
    for (int i=0; i<m_robot->numJoints(); i++){
        pose[i] = mask[i] ? angles[i] : m_qInit.data[i];
    }
    m_seq->setJointAngles(pose, tm);
    return true;
}

bool SequencePlayer::setJointAnglesSequence(const OpenHRP::dSequenceSequence angless, const OpenHRP::bSequence& mask, const OpenHRP::dSequence& times)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);

    if (!setInitialState()) return false;

    bool tmp_mask[robot()->numJoints()];
    if (mask.length() != robot()->numJoints()) {
        for (int i=0; i < robot()->numJoints(); i++) tmp_mask[i] = true;
    }else{
        for (int i=0; i < robot()->numJoints(); i++) tmp_mask[i] = mask.get_buffer()[i];
    }
    int len = angless.length();
    std::vector<const double*> v_poss;
    std::vector<double> v_tms;
    for ( int i = 0; i < angless.length(); i++ ) v_poss.push_back(angless[i].get_buffer());
    for ( int i = 0; i <  times.length();  i++ )  v_tms.push_back(times[i]);
    return m_seq->setJointAnglesSequence(v_poss, v_tms);
}

bool SequencePlayer::clearJointAngles()
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);

    if (!setInitialState()) return false;

    return m_seq->clearJointAngles();
}

bool SequencePlayer::setJointAnglesSequenceOfGroup(const char *gname, const OpenHRP::dSequenceSequence angless, const OpenHRP::dSequence& times)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);
    if (!setInitialState()) return false;

    if (!m_seq->resetJointGroup(gname, m_qInit.data.get_buffer())) return false;

    std::vector<const double*> v_poss;
    std::vector<double> v_tms;
    for ( int i = 0; i < angless.length(); i++ ) v_poss.push_back(angless[i].get_buffer());
    for ( int i = 0; i <  times.length();  i++ )  v_tms.push_back(times[i]);
    return m_seq->setJointAnglesSequenceOfGroup(gname, v_poss, v_tms, angless.length()>0?angless[0].length():0);
}

bool SequencePlayer::clearJointAnglesOfGroup(const char *gname)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);
    if (!setInitialState()) return false;

    if (!m_seq->resetJointGroup(gname, m_qInit.data.get_buffer())) return false;

    return m_seq->clearJointAnglesOfGroup(gname);
}

bool SequencePlayer::setJointAnglesSequenceFull(const OpenHRP::dSequenceSequence i_jvss, const OpenHRP::dSequenceSequence i_vels, const OpenHRP::dSequenceSequence i_torques, const OpenHRP::dSequenceSequence i_poss, const OpenHRP::dSequenceSequence i_rpys, const OpenHRP::dSequenceSequence i_accs, const OpenHRP::dSequenceSequence i_zmps, const OpenHRP::dSequenceSequence i_wrenches, const OpenHRP::dSequenceSequence i_optionals, const dSequence i_tms)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);

    if (!setInitialState()) return false;

    int len = i_jvss.length();
    std::vector<const double*> v_jvss, v_vels, v_torques, v_poss, v_rpys, v_accs, v_zmps, v_wrenches, v_optionals;
    std::vector<double> v_tms;
    for ( int i = 0; i < i_jvss.length(); i++ ) v_jvss.push_back(i_jvss[i].get_buffer());
    for ( int i = 0; i < i_vels.length(); i++ ) v_vels.push_back(i_vels[i].get_buffer());
    for ( int i = 0; i < i_torques.length(); i++ ) v_torques.push_back(i_torques[i].get_buffer());
    for ( int i = 0; i < i_poss.length(); i++ ) v_poss.push_back(i_poss[i].get_buffer());
    for ( int i = 0; i < i_rpys.length(); i++ ) v_rpys.push_back(i_rpys[i].get_buffer());
    for ( int i = 0; i < i_accs.length(); i++ ) v_accs.push_back(i_accs[i].get_buffer());
    for ( int i = 0; i < i_zmps.length(); i++ ) v_zmps.push_back(i_zmps[i].get_buffer());
    for ( int i = 0; i < i_wrenches.length(); i++ ) v_wrenches.push_back(i_wrenches[i].get_buffer());
    for ( int i = 0; i < i_optionals.length(); i++ ) v_optionals.push_back(i_optionals[i].get_buffer());
    for ( int i = 0; i < i_tms.length();  i++ )  v_tms.push_back(i_tms[i]);
    return m_seq->setJointAnglesSequenceFull(v_jvss, v_vels, v_torques, v_poss, v_rpys, v_accs, v_zmps, v_wrenches, v_optionals, v_tms);
}

bool SequencePlayer::setBasePos(const double *pos, double tm)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);
    m_seq->setBasePos(pos, tm);
    return true;
}

bool SequencePlayer::setBaseRpy(const double *rpy, double tm)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);
    m_seq->setBaseRpy(rpy, tm);
    return true;
}

bool SequencePlayer::setZmp(const double *zmp, double tm)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);
    m_seq->setZmp(zmp, tm);
    return true;
}

bool SequencePlayer::setWrenches(const double *wrenches, double tm)
{
    Guard guard(m_mutex);
    m_seq->setWrenches(wrenches, tm);
    return true;
}

bool SequencePlayer::setTargetPose(const char* gname, const double *xyz, const double *rpy, double tm, const char* frame_name)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);
    if (!setInitialState()) return false;
    // setup
    std::vector<int> indices;
    hrp::dvector start_av, end_av;
    std::vector<hrp::dvector> avs;
    if (! m_seq->getJointGroup(gname, indices) ) {
        std::cerr << "[setTargetPose] Could not find joint group " << gname << std::endl;
        return false;
    }
    start_av.resize(indices.size());
    end_av.resize(indices.size());

    //std::cerr << std::endl;
    if ( ! m_robot->joint(indices[0])->parent ) {
        std::cerr << "[setTargetPose] " << m_robot->joint(indices[0])->name << " does not have parent" << std::endl;
        return false;
    }
    string base_parent_name = m_robot->joint(indices[0])->parent->name;
    string target_name = m_robot->joint(indices[indices.size()-1])->name;
    // prepare joint path
    hrp::JointPathExPtr manip = hrp::JointPathExPtr(new hrp::JointPathEx(m_robot, m_robot->link(base_parent_name), m_robot->link(target_name), dt, true, std::string(m_profile.instance_name)));

    // calc fk
    for (int i=0; i<m_robot->numJoints(); i++){
        hrp::Link *j = m_robot->joint(i);
        if (j) j->q = m_qRef.data.get_buffer()[i];
    }
    m_robot->calcForwardKinematics();
    for ( int i = 0; i < manip->numJoints(); i++ ){
        start_av[i] = manip->joint(i)->q;
    }

    // xyz and rpy are relateive to root link, where as pos and rotatoin of manip->calcInverseKinematics are relative to base link

    // ik params
    hrp::Vector3 start_p(m_robot->link(target_name)->p);
    hrp::Matrix33 start_R(m_robot->link(target_name)->R);
    hrp::Vector3 end_p(xyz[0], xyz[1], xyz[2]);
    hrp::Matrix33 end_R = m_robot->link(target_name)->calcRfromAttitude(hrp::rotFromRpy(rpy[0], rpy[1], rpy[2]));

    // change start and end must be relative to the frame_name
    if ( (frame_name != NULL) && (! m_robot->link(frame_name) ) ) {
        std::cerr << "[setTargetPose] Could not find frame_name " << frame_name << std::endl;
        return false;
    } else if ( frame_name != NULL ) {
        hrp::Vector3 frame_p(m_robot->link(frame_name)->p);
        hrp::Matrix33 frame_R(m_robot->link(frame_name)->attitude());
        // fix start/end references from root to frame;
        end_p = frame_R * end_p + frame_p;
        end_R = frame_R * end_R;
    }
    manip->setMaxIKError(m_error_pos,m_error_rot);
    manip->setMaxIKIteration(m_iteration);
    std::cerr << "[setTargetPose] Solveing IK with frame" << frame_name << ", Error " << m_error_pos << m_error_rot << ", Iteration " << m_iteration << std::endl;
    std::cerr << "                Start " << start_p << start_R<< std::endl;
    std::cerr << "                End   " << end_p << end_R<< std::endl;

    // interpolate & calc ik
    int len = max(((start_p - end_p).norm() / 0.02 ), // 2cm
                  ((hrp::omegaFromRot(start_R.transpose() * end_R).norm()) / 0.025)); // 2 deg
    len = max(len, 1);

    std::vector<const double *> v_pos;
    std::vector<double> v_tm;
    v_pos.resize(len);
    v_tm.resize(len);

    // do loop
    for (int i = 0; i < len; i++ ) {
        double a = (1+i)/(double)len;
        hrp::Vector3 p = (1-a)*start_p + a*end_p;
        hrp::Vector3 omega = hrp::omegaFromRot(start_R.transpose() * end_R);
        hrp::Matrix33 R = start_R * rodrigues(omega.isZero()?omega:omega.normalized(), a*omega.norm());
        bool ret = manip->calcInverseKinematics2(p, R);

        if ( m_debugLevel > 0 ) {
            // for debug
            std::cerr << "target pos/rot : " << i << "/" << a << " : "
                      << p[0] << " " << p[1] << " " << p[2] << ","
                      << omega[0] << " " << omega[1] << " " << omega[2] << std::endl;
        }
        if ( ! ret ) {
            std::cerr << "[setTargetPose] IK failed" << std::endl;
            return false;
        }
        v_pos[i] = (const double *)malloc(sizeof(double)*manip->numJoints());
        for ( int j = 0; j < manip->numJoints(); j++ ){
            ((double *)v_pos[i])[j] = manip->joint(j)->q;
        }
        v_tm[i] = tm/len;
    }

    if ( m_debugLevel > 0 ) {
        // for debug
        for(int i = 0; i < len; i++ ) {
            std::cerr << v_tm[i] << ":";
            for(int j = 0; j < start_av.size(); j++ ) {
                std::cerr << v_pos[i][j] << " ";
            }
            std::cerr << std::endl;
        }
    }

    bool ret = m_seq->playPatternOfGroup(gname, v_pos, v_tm, m_qInit.data.get_buffer(), v_pos.size()>0?indices.size():0);

    // clean up memory, need to improve
    for (int i = 0; i < len; i++ ) {
        free((double *)v_pos[i]);
    }

    return ret;
}

void SequencePlayer::loadPattern(const char *basename, double tm)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);
    if (setInitialState()){
        m_seq->loadPattern(basename, tm);
    }
}

bool SequencePlayer::setInitialState(double tm)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << "m_seq-isEmpty() " << m_seq->isEmpty() << ", m_Init.data.length() " << m_qInit.data.length() << std::endl;
    }
    if (!m_seq->isEmpty()) return true;

    if (m_qInit.data.length() == 0){
        std::cerr << "can't determine initial posture" << std::endl;
        return false;
    }else{
        m_seq->setJointAngles(m_qInit.data.get_buffer(), tm);
        for (int i=0; i<m_robot->numJoints(); i++){
            Link *l = m_robot->joint(i);
            l->q = m_qInit.data[i];
            m_qRef.data[i] = m_qInit.data[i]; // update m_qRef for setTargetPose()
        }

        Link *root = m_robot->rootLink();

        root->p << m_basePosInit.data.x,
            m_basePosInit.data.y,
            m_basePosInit.data.z;
        m_seq->setBasePos(root->p.data(), tm);

        double rpy[] = {m_baseRpyInit.data.r,
                        m_baseRpyInit.data.p,
                        m_baseRpyInit.data.y};
        m_seq->setBaseRpy(rpy, tm);
        calcRotFromRpy(root->R, rpy[0], rpy[1], rpy[2]);

        double zmp[] = {m_zmpRefInit.data.x, m_zmpRefInit.data.y, m_zmpRefInit.data.z};
        m_seq->setZmp(zmp, tm);
        double zero[] = {0,0,0};
        m_seq->setBaseAcc(zero, tm);
        return true;
    }
}

void SequencePlayer::playPattern(const dSequenceSequence& pos, const dSequenceSequence& rpy, const dSequenceSequence& zmp, const dSequence& tm)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);
    if (!setInitialState()) return;

    std::vector<const double *> v_pos, v_rpy, v_zmp;
    std::vector<double> v_tm;
    for ( int i = 0; i < pos.length(); i++ ) v_pos.push_back(pos[i].get_buffer());
    for ( int i = 0; i < rpy.length(); i++ ) v_rpy.push_back(rpy[i].get_buffer());
    for ( int i = 0; i < zmp.length(); i++ ) v_zmp.push_back(zmp[i].get_buffer());
    for ( int i = 0; i < tm.length() ; i++ ) v_tm.push_back(tm[i]);
    return m_seq->playPattern(v_pos, v_rpy, v_zmp, v_tm, m_qInit.data.get_buffer(), pos.length()>0?pos[0].length():0);
}

bool SequencePlayer::setInterpolationMode(OpenHRP::SequencePlayerService::interpolationMode i_mode_)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);
    interpolator::interpolation_mode new_mode;
    if (i_mode_ == OpenHRP::SequencePlayerService::LINEAR){
        new_mode = interpolator::LINEAR;
    }else if (i_mode_ == OpenHRP::SequencePlayerService::HOFFARBIB){
        new_mode = interpolator::HOFFARBIB;
    }else{
        return false;
    }
    return m_seq->setInterpolationMode(new_mode);
}

bool SequencePlayer::addJointGroup(const char *gname, const OpenHRP::SequencePlayerService::StrSequence& jnames)
{
    std::cerr << "[addJointGroup] group name = " << gname << std::endl;
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    if (!waitInterpolationOfGroup(gname)) return false;

    Guard guard(m_mutex);
    std::vector<int> indices;
    for (size_t i=0; i<jnames.length(); i++){
        hrp::Link *l = m_robot->link(std::string(jnames[i]));
        if (l){
            indices.push_back(l->jointId);
        }else{
            std::cerr << "[addJointGroup] link name " << jnames[i] << "is not found" << std::endl;
            return false;
        }
    }
    return m_seq->addJointGroup(gname, indices);
}

bool SequencePlayer::removeJointGroup(const char *gname)
{
    std::cerr << "[removeJointGroup] group name = " << gname << std::endl;
    if (!waitInterpolationOfGroup(gname)) return false;
    bool ret;
    {
        Guard guard(m_mutex);
        ret = m_seq->removeJointGroup(gname);
    }
    return ret;
}

bool SequencePlayer::setJointAnglesOfGroup(const char *gname, const dSequence& jvs, double tm)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);
    if (!setInitialState()) return false;

    if (!m_seq->resetJointGroup(gname, m_qInit.data.get_buffer())) return false;
    return m_seq->setJointAnglesOfGroup(gname, jvs.get_buffer(), jvs.length(), tm);
}

bool SequencePlayer::playPatternOfGroup(const char *gname, const dSequenceSequence& pos, const dSequence& tm)
{
    if ( m_debugLevel > 0 ) {
        std::cerr << __PRETTY_FUNCTION__ << std::endl;
    }
    Guard guard(m_mutex);
    if (!setInitialState()) return false;

    std::vector<const double *> v_pos;
    std::vector<double> v_tm;
    for ( int i = 0; i < pos.length(); i++ ) v_pos.push_back(pos[i].get_buffer());
    for ( int i = 0; i < tm.length() ; i++ ) v_tm.push_back(tm[i]);
    return m_seq->playPatternOfGroup(gname, v_pos, v_tm, m_qInit.data.get_buffer(), pos.length()>0?pos[0].length():0);
}

void SequencePlayer::setMaxIKError(double pos, double rot){
    m_error_pos = pos;
    m_error_rot = rot;
}

void SequencePlayer::setMaxIKIteration(short iter){
    m_iteration= iter;
}


extern "C"
{

    void SequencePlayerInit(RTC::Manager* manager)
    {
        RTC::Properties profile(sequenceplayer_spec);
        manager->registerFactory(profile,
                                 RTC::Create<SequencePlayer>,
                                 RTC::Delete<SequencePlayer>);
    }

};



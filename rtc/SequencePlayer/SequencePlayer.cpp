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
#include "VectorConvert.h"

// Module specification
// <rtc-template block="module_spec">
static const char* sequenceplayer_spec[] =
    {
        "implementation_id", "SequencePlayer",
        "type_name",         "SequencePlayer",
        "description",       "sequence player component",
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

SequencePlayer::SequencePlayer(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      // <rtc-template block="initializer">
      m_qInitIn("qInit", m_qInit),
      m_basePosInitIn("basePosInit", m_basePosInit),
      m_baseRpyInitIn("baseRpyInit", m_baseRpyInit),
      m_qRefOut("qRef", m_qRef),
      m_zmpRefOut("zmpRef", m_zmpRef),
      m_accRefOut("accRef", m_accRef),
      m_basePosOut("basePos", m_basePos),
      m_baseRpyOut("baseRpy", m_baseRpy),
      m_SequencePlayerServicePort("SequencePlayerService"),
      // </rtc-template>
      dummy(0),
      m_robot(NULL)
{
    m_service0.player(this);
    m_clearFlag = false;
    m_waitFlag = false;
    sem_init(&m_waitSem, 0, 0);
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
  
    // Set OutPort buffer
    addOutPort("qRef", m_qRefOut);
    addOutPort("zmpRef", m_zmpRefOut);
    addOutPort("accRef", m_accRefOut);
    addOutPort("basePos", m_basePosOut);
    addOutPort("baseRpy", m_baseRpyOut);
  
    // Set service provider to Ports
    m_SequencePlayerServicePort.registerProvider("service0", "SequencePlayerService", m_service0);
  
    // Set service consumers to Ports
  
    // Set CORBA Service Ports
    addPort(m_SequencePlayerServicePort);
  
    // </rtc-template>
    // <rtc-template block="bind_config">
    // Bind variables and configuration variable
  
    // </rtc-template>

    RTC::Properties& prop = getProperties();
    double dt;
    coil::stringTo(dt, prop["dt"].c_str());

    m_robot = new Body();

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

    m_seq = new seqplay(dof, dt);

    m_qInit.data.length(dof);
    for (unsigned int i=0; i<dof; i++) m_qInit.data[i] = 0.0;
    m_basePosInit.data.x = m_basePosInit.data.y = m_basePosInit.data.z = 0.0; 
    m_baseRpyInit.data.r = m_baseRpyInit.data.p = m_baseRpyInit.data.y = 0.0;

    // allocate memory for outPorts
    m_qRef.data.length(dof);

    return RTC::RTC_OK;
}



RTC::ReturnCode_t SequencePlayer::onFinalize()
{
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
    //std::cout << "SequencePlayer::onExecute(" << ec_id << ")" << std::endl;
    if (m_qInitIn.isNew()) m_qInitIn.read();
    if (m_basePosInitIn.isNew()) m_basePosInitIn.read();
    if (m_baseRpyInitIn.isNew()) m_baseRpyInitIn.read();
    
    if (m_seq->isEmpty()){
        m_clearFlag = false;
        if (m_waitFlag){
            sem_post(&m_waitSem);
            m_waitFlag = false;
        }
    }else{
        double zmp[3], acc[3], pos[3], rpy[3];
        m_seq->get(m_qRef.data.get_buffer(), zmp, acc, pos, rpy);
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
        m_qRefOut.write();
        m_zmpRefOut.write();
        m_accRefOut.write();
        m_basePosOut.write();
        m_baseRpyOut.write();

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
    m_clearFlag = true;
}

void SequencePlayer::waitInterpolation()
{
    m_waitFlag = true;
    sem_wait(&m_waitSem);
}

bool SequencePlayer::setJointAngle(short id, double angle, double tm)
{
    if (!setInitialState()) return false;
    m_seq->setJointAngle(id, angle, tm);
    return true;
}

bool SequencePlayer::setJointAngles(const double *angles, double tm)
{
    if (!setInitialState()) return false;
    m_seq->setJointAngles(angles, tm);
    return true;
}

bool SequencePlayer::setJointAngles(const double *angles, const bool *mask, 
                                    double tm)
{
    if (!setInitialState()) return false;
    double pose[m_robot->numJoints()];
    for (int i=0; i<m_robot->numJoints(); i++){
        pose[i] = mask[i] ? angles[i] : m_qInit.data[i];
    }
    m_seq->setJointAngles(pose, tm);
    return true;
}

bool SequencePlayer::setBasePos(const double *pos, double tm)
{
    m_seq->setBasePos(pos, tm);
    return true;
}

bool SequencePlayer::setBaseRpy(const double *rpy, double tm)
{
    m_seq->setBaseRpy(rpy, tm);
    return true;
}

bool SequencePlayer::setZmp(const double *zmp, double tm)
{
    m_seq->setZmp(zmp, tm);
    return true;
}

void SequencePlayer::loadPattern(const char *basename, double tm)
{
    if (setInitialState()){
        m_seq->loadPattern(basename, tm);
    }
}

bool SequencePlayer::setInitialState()
{
    if (!m_seq->isEmpty()) return true;

    if (m_qInit.data.length() == 0){
        std::cerr << "can't determine initial posture" << std::endl;
        return false;
    }else{
        m_seq->setJointAngles(m_qInit.data.get_buffer());
        for (int i=0; i<m_robot->numJoints(); i++){
            Link *l = m_robot->joint(i);
            l->q = m_qInit.data[i];
        }

        Link *root = m_robot->rootLink();

        root->p = m_basePosInit.data.x,
            m_basePosInit.data.y,
            m_basePosInit.data.z;
        m_seq->setBasePos(root->p.data());

        double rpy[] = {m_baseRpyInit.data.r,
                        m_baseRpyInit.data.p,
                        m_baseRpyInit.data.y};
        m_seq->setBaseRpy(rpy);
        calcRotFromRpy(root->R, rpy[0], rpy[1], rpy[2]);

#if 0
        m_robot->calcForwardKinematics();
        Vector3 com = m_robot->calcCM();
        com[2] = 0.0;
        Vector3 local_com(trans(root->R)*(com - root->p));
        double zmp[] = {local_com[0], local_com[1], local_com[2]};
        m_seq->setZmp(zmp);
#endif
        double zero[] = {0,0,0};
        m_seq->setBaseAcc(zero);
        return true;
    }
}

void SequencePlayer::playPattern(const dSequenceSequence& pos, const dSequenceSequence& rpy, const dSequenceSequence& zmp, ::CORBA::Double tm)
{
    if (!setInitialState()) return;

    const double *q=NULL, *z=NULL, *a=NULL, *p=NULL, *e=NULL;
    for (unsigned int i=0; i<pos.length(); i++){
        q = pos[i].get_buffer();
        if (i < zmp.length()) z = zmp[i].get_buffer();
        if (i < rpy.length()) e = rpy[i].get_buffer();
        if (i==0){
            m_seq->go(q, z, a, p, e, tm, false);
        }else{
            m_seq->push(q, z, a, p, e, false);
        }
    }
    m_seq->sync();
}

bool SequencePlayer::setInterpolationMode(OpenHRP::SequencePlayerService::interpolationMode i_mode_)
{
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



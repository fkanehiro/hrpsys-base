// -*- C++ -*-
/*!
 * @file  CollisionDetector.cpp
 * @brief collisoin detector component
 * $Date$
 *
 * $Id$
 */

#include <iomanip>
#include <rtm/CorbaNaming.h>
#include <hrpModel/Link.h>
#include <hrpModel/JointPath.h>
#include <hrpUtil/Eigen3d.h>
#include <hrpUtil/Eigen4d.h>
#include <hrpCollision/ColdetModel.h>
#include "util/GLbody.h"
#include "util/GLutil.h"
#include "util/BVutil.h"

#include "CollisionDetector.h"

#define deg2rad(x)	((x)*M_PI/180)
#define rad2deg(x)      ((x)*180/M_PI)

// Module specification
// <rtc-template block="module_spec">
static const char* component_spec[] =
{
    "implementation_id", "CollisionDetector",
    "type_name",         "CollisionDetector",
    "description",       "collisoin detector component",
    "version",           "1.0",
    "vendor",            "AIST",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.project", "",
    ""
};
// </rtc-template>

CollisionDetector::CollisionDetector(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      // <rtc-template block="initializer">
      m_qIn("q", m_q),
      m_qRefOut("qRef", m_qRef),
      // </rtc-template>
      m_glbody(NULL),
      use_viewer(false),
      m_robot(hrp::BodyPtr()),
      m_scene(&m_log),
      m_window(&m_scene, &m_log),
      dummy(0)
{
    m_log.enableRingBuffer(1);
}

CollisionDetector::~CollisionDetector()
{
}



RTC::ReturnCode_t CollisionDetector::onInitialize()
{
    std::cout << m_profile.instance_name << ": onInitialize()" << std::endl;
    // <rtc-template block="bind_config">
    // Bind variables and configuration variable
  
    // </rtc-template>

    // Registration: InPort/OutPort/Service
    // <rtc-template block="registration">
    // Set InPort buffers
    addInPort("q", m_qIn);

    // Set OutPort buffer
    addOutPort("qRef", m_qRefOut);
  
    // Set service provider to Ports
  
    // Set service consumers to Ports
  
    // Set CORBA Service Ports
  
    // </rtc-template>

    //RTC::Properties& prop = getProperties();

    return RTC::RTC_OK;
}



/*
  RTC::ReturnCode_t CollisionDetector::onFinalize()
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t CollisionDetector::onStartup(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t CollisionDetector::onShutdown(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

RTC::ReturnCode_t CollisionDetector::onActivated(RTC::UniqueId ec_id)
{
    std::cout << m_profile.instance_name<< ": onActivated(" << ec_id << ")" << std::endl;

    RTC::Manager& rtcManager = RTC::Manager::instance();
    std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
    int comPos = nameServer.find(",");
    if (comPos < 0){
        comPos = nameServer.length();
    }
    nameServer = nameServer.substr(0, comPos);
    RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());

    RTC::Properties& prop = getProperties();

    if ( prop["collision_viewer"] == "true" ) {
	use_viewer = true;
    }

    m_glbody = new GLbody();
    m_robot = hrp::BodyPtr(m_glbody);
    OpenHRP::BodyInfo_var binfo;
    binfo = hrp::loadBodyInfo(prop["model"].c_str(),
			      CosNaming::NamingContext::_duplicate(naming.getRootContext()));
    if (CORBA::is_nil(binfo)){
	std::cerr << "failed to load model[" << prop["model"] << "]"
		  << std::endl;
	return RTC::RTC_ERROR;
    }
    if (!loadBodyFromBodyInfo(m_robot, binfo, true, GLlinkFactory)) {
	std::cerr << "failed to load model[" << prop["model"] << "]" << std::endl;
	return RTC::RTC_ERROR;
    }
    loadShapeFromBodyInfo(m_glbody, binfo);
    convertToConvexHull(m_robot);

    if ( prop["collision_pair"] != "" ) {
	std::cerr << "prop[collision_pair] ->" << prop["collision_pair"] << std::endl;
	std::istringstream iss(prop["collision_pair"]);
	std::string tmp;
	while (getline(iss, tmp, ' ')) {
	    size_t pos = tmp.find_first_of(':');
	    std::string name1 = tmp.substr(0, pos), name2 = tmp.substr(pos+1);
	    std::cerr << "check collisions between " << m_robot->link(name1)->name << " and " <<  m_robot->link(name2)->name << std::endl;
	    m_pair.push_back(new hrp::ColdetLinkPair(m_robot->link(name1), m_robot->link(name2)));
	}
    }

    if ( m_pair.size() == 0 ) {
	std::cerr << "failed to setup collisions" << std::endl;
	return RTC::RTC_ERROR;
    }

    m_scene.addBody(m_robot);

    // allocate memory for outPorts
    m_qRef.data.length(0);

    return RTC::RTC_OK;
}

RTC::ReturnCode_t CollisionDetector::onDeactivated(RTC::UniqueId ec_id)
{
    std::cout << m_profile.instance_name<< ": onDeactivated(" << ec_id << ")" << std::endl;

    return RTC::RTC_OK;
}

RTC::ReturnCode_t CollisionDetector::onExecute(RTC::UniqueId ec_id)
{
    if (m_qIn.isNew()) {
	m_qIn.read();

	assert(m_q.data.length() == m_robot->numJoints());
        for (int i=0; i<m_glbody->numLinks(); i++){
            ((GLlink *)m_glbody->link(i))->highlight(false);
        }
        
	double posture[m_q.data.length()];
	for ( int i = 0; i < m_robot->numJoints(); i++ ){
	    m_robot->joint(i)->q = posture[i] = m_q.data[i];
	}
	m_robot->calcForwardKinematics();
	m_robot->updateLinkColdetModelPositions();

	bool safe_posture = true;
	coil::TimeValue tm1 = coil::gettimeofday();
	for (unsigned int i = 0; i < m_pair.size(); i++){
	    hrp::ColdetLinkPairPtr p = m_pair[i];
#if 1
	    if ( p->detectIntersection() ) {
		safe_posture = false;
		hrp::JointPathPtr jointPath = m_robot->getJointPath(p->link(0),p->link(1));
		std::cerr << i << "/" << m_pair.size() << " pair: " << p->link(0)->name << "/" << p->link(1)->name << "(" << jointPath->numJoints() << ")" << std::endl;
                ((GLlink *)p->link(0))->highlight(true);
                ((GLlink *)p->link(1))->highlight(true);
	    }
#else
	    double point0[3], point1[3];
	    double d = p->computeDistance(point0, point1);
	    if ( d <= 0.05 ) {
		safe_posture = false;
		hrp::JointPathPtr jointPath = m_robot->getJointPath(p->link(0),p->link(1));
		std::cerr << i << "/" << m_pair.size() << " pair: " << p->link(0)->name << "/" << p->link(1)->name << "(" << jointPath->numJoints() << "), distance = " << d << std::endl;
	    }
#endif
	}
	coil::TimeValue tm2 = coil::gettimeofday();
	std::cerr << "check collisions for for " << m_pair.size() << " pairs in " << (tm2.sec()-tm1.sec())*1000+(tm2.usec()-tm1.usec())/1000 << "[msec]" << std::endl;

        TimedPosture tp;
        tp.time = 0;
        tp.posture.resize(m_q.data.length());
        for (size_t i=0; i<tp.posture.size(); i++) tp.posture[i] = m_q.data[i];
        m_log.add(tp);

	if ( safe_posture ) {
	    if ( m_qRef.data.length() == 0 ) { // not initialized
		m_qRef.data.length(m_q.data.length());
	    }
	    for( int i = 0; i < m_qRef.data.length(); i++ ) {
		m_qRef.data[i] = m_q.data[i];
	    }
	}


	if ( m_qRef.data.length() != 0 ) { // initialized
	    m_qRefOut.write();
	}
    }

    if ( use_viewer ) m_window.oneStep();

    return RTC::RTC_OK;
}

/*
  RTC::ReturnCode_t CollisionDetector::onAborting(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t CollisionDetector::onError(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t CollisionDetector::onReset(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t CollisionDetector::onStateUpdate(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/

/*
  RTC::ReturnCode_t CollisionDetector::onRateChanged(RTC::UniqueId ec_id)
  {
  return RTC::RTC_OK;
  }
*/



extern "C"
{

    void CollisionDetectorInit(RTC::Manager* manager)
    {
        RTC::Properties profile(component_spec);
        manager->registerFactory(profile,
                                 RTC::Create<CollisionDetector>,
                                 RTC::Delete<CollisionDetector>);
    }

};



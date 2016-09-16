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
#ifdef USE_HRPSYSUTIL
#include "hrpsys/util/GLbody.h"
#include "hrpsys/util/GLutil.h"
#endif // USE_HRPSYSUTIL
#include "hrpsys/util/BVutil.h"
#include "hrpsys/idl/RobotHardwareService.hh"

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

CollisionDetector::CollisionDetector(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      // <rtc-template block="initializer">
      m_qRefIn("qRef", m_qRef),
      m_qCurrentIn("qCurrent", m_qCurrent),
      m_servoStateIn("servoStateIn", m_servoState),
      m_qOut("q", m_q),
      m_beepCommandOut("beepCommand", m_beepCommand),
      m_CollisionDetectorServicePort("CollisionDetectorService"),
      // </rtc-template>
      m_loop_for_check(0),
      m_collision_loop(1),
      m_use_limb_collision(false),
#ifdef USE_HRPSYSUTIL
      m_glbody(NULL),
#endif // USE_HRPSYSUTIL
      m_use_viewer(false),
      m_robot(hrp::BodyPtr()),
#ifdef USE_HRPSYSUTIL
      m_scene(&m_log),
      m_window(&m_scene, &m_log),
#endif // USE_HRPSYSUTIL
      m_debugLevel(0),
      m_enable(true),
      collision_beep_count(0),
      dummy(0),
      is_beep_port_connected(false)
{
    m_service0.collision(this);
#ifdef USE_HRPSYSUTIL
    m_log.enableRingBuffer(1);
#endif // USE_HRPSYSUTIL
    init_beep();
    start_beep(3136);
}

CollisionDetector::~CollisionDetector()
{
  quit_beep();
}



RTC::ReturnCode_t CollisionDetector::onInitialize()
{
    std::cerr << "[" << m_profile.instance_name << "] onInitialize()" << std::endl;
    // <rtc-template block="bind_config">
    // Bind variables and configuration variable
    bindParameter("debugLevel", m_debugLevel, "0");
  
    // </rtc-template>

    // Registration: InPort/OutPort/Service
    // <rtc-template block="registration">
    // Set InPort buffers
    addInPort("qRef", m_qRefIn);
    addInPort("qCurrent", m_qCurrentIn);
    addInPort("servoStateIn", m_servoStateIn);

    // Set OutPort buffer
    addOutPort("q", m_qOut);
    addOutPort("beepCommand", m_beepCommandOut);
  
    // Set service provider to Ports
    m_CollisionDetectorServicePort.registerProvider("service0", "CollisionDetectorService", m_service0);
  
    // Set service consumers to Ports
  
    // Set CORBA Service Ports
    addPort(m_CollisionDetectorServicePort);
  
    // </rtc-template>

    //RTC::Properties& prop = getProperties();

    RTC::Manager& rtcManager = RTC::Manager::instance();
    std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
    int comPos = nameServer.find(",");
    if (comPos < 0){
        comPos = nameServer.length();
    }
    nameServer = nameServer.substr(0, comPos);
    RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());

    RTC::Properties& prop = getProperties();

    coil::stringTo(m_dt, prop["dt"].c_str());

    if ( prop["collision_viewer"] == "true" ) {
	m_use_viewer = true;
    }
#ifdef USE_HRPSYSUTIL
    m_glbody = new GLbody();
    m_robot = hrp::BodyPtr(m_glbody);
#else
    m_robot = hrp::BodyPtr(new hrp::Body());
#endif // USE_HRPSYSUTIL
    //
    OpenHRP::BodyInfo_var binfo;
    binfo = hrp::loadBodyInfo(prop["model"].c_str(),
			      CosNaming::NamingContext::_duplicate(naming.getRootContext()));
    if (CORBA::is_nil(binfo)){
	std::cerr << "[" << m_profile.instance_name << "] failed to load model[" << prop["model"] << "]"
		  << std::endl;
	return RTC::RTC_ERROR;
    }
#ifdef USE_HRPSYSUTIL
    if (!loadBodyFromBodyInfo(m_robot, binfo, true, GLlinkFactory)) {
#else
    if (!loadBodyFromBodyInfo(m_robot, binfo, true, hrplinkFactory)) {
#endif // USE_HRPSYSUTIL
      std::cerr << "[" << m_profile.instance_name << "] failed to load model[" << prop["model"] << "] in "
                << m_profile.instance_name << std::endl;
      return RTC::RTC_ERROR;
    }
#ifdef USE_HRPSYSUTIL
    loadShapeFromBodyInfo(m_glbody, binfo);
#endif // USE_HRPSYSUTIL
    if ( prop["collision_model"] == "AABB" ) {
        convertToAABB(m_robot);
    } else if ( prop["collision_model"] == "convex hull" ||
                prop["collision_model"] == "" ) { // set convex hull as default
        convertToConvexHull(m_robot);
    }
    setupVClipModel(m_robot);

    if ( prop["collision_pair"] != "" ) {
	std::cerr << "[" << m_profile.instance_name << "] prop[collision_pair] ->" << prop["collision_pair"] << std::endl;
	std::istringstream iss(prop["collision_pair"]);
	std::string tmp;
	while (getline(iss, tmp, ' ')) {
	    size_t pos = tmp.find_first_of(':');
	    std::string name1 = tmp.substr(0, pos), name2 = tmp.substr(pos+1);
            if ( m_robot->link(name1)==NULL ) {
                std::cerr << "[" << m_profile.instance_name << "] Could not find robot link " << name1 << std::endl;
		std::cerr << "[" << m_profile.instance_name << "] please choose one of following :";
		for (int i=0; i < m_robot->numLinks(); i++) {
		  std::cerr << " " << m_robot->link(i)->name;
		}
		std::cerr << std::endl;
                continue;
            }
            if ( m_robot->link(name2)==NULL ) {
                std::cerr << "[" << m_profile.instance_name << "] Could not find robot link " << name2 << std::endl;
		std::cerr << "[" << m_profile.instance_name << "] please choose one of following :";
		for (int i=0; i < m_robot->numLinks(); i++) {
		  std::cerr << " " << m_robot->link(i)->name;
		}
		std::cerr << std::endl;
                continue;
            }
	    std::cerr << "[" << m_profile.instance_name << "] check collisions between " << m_robot->link(name1)->name << " and " <<  m_robot->link(name2)->name << std::endl;
	    m_pair[tmp] = new CollisionLinkPair(new VclipLinkPair(m_robot->link(name1), m_VclipLinks[m_robot->link(name1)->index],
                                                                  m_robot->link(name2), m_VclipLinks[m_robot->link(name2)->index], 0));
	}
    }

    if ( prop["collision_loop"] != "" ) {
        coil::stringTo(m_collision_loop, prop["collision_loop"].c_str());
        std::cerr << "[" << m_profile.instance_name << "] set collision_loop: " << m_collision_loop << std::endl;
    }
#ifdef USE_HRPSYSUTIL
    if ( m_use_viewer ) {
      m_scene.addBody(m_robot);
      GLlink::drawMode(GLlink::DM_COLLISION);
    }
#endif // USE_HRPSYSUTIL

    // true (1) do not move when collide,
    // false(0) move even if collide
    m_curr_collision_mask.resize(m_robot->numJoints()); // collision_mask used to select output                0: passthough reference data, 1 output safe data
    std::fill(m_curr_collision_mask.begin(), m_curr_collision_mask.end(), 0);
    m_init_collision_mask.resize(m_robot->numJoints()); // collision_mask defined in .conf as [collisoin_mask] 0: move even if collide, 1: do not move when collide
    std::fill(m_init_collision_mask.begin(), m_init_collision_mask.end(), 1);
    if ( prop["collision_mask"] != "" ) {
	std::cerr << "[" << m_profile.instance_name << "] prop[collision_mask] ->" << prop["collision_mask"] << std::endl;
        coil::vstring mask_str = coil::split(prop["collision_mask"], ",");
        if (mask_str.size() == m_robot->numJoints()) {
            for (size_t i = 0; i < m_robot->numJoints(); i++) {coil::stringTo(m_init_collision_mask[i], mask_str[i].c_str()); }
            for (size_t i = 0; i < m_robot->numJoints(); i++) {
                if ( m_init_collision_mask[i] == 0 ) {
                    std::cerr << "[" << m_profile.instance_name << "] CollisionDetector will not control " << m_robot->joint(i)->name << std::endl;
                }
            }
        }else{
            std::cerr << "[" << m_profile.instance_name << "] ERROR size of collision_mask is differ from robot joint number .. " << mask_str.size()  << ", " << m_robot->numJoints() << std::endl;
        }
    }

    if ( prop["use_limb_collision"] != "" ) {
        std::cerr << "[" << m_profile.instance_name << "] prop[use_limb_collision] -> " << prop["use_limb_collision"] << std::endl;
        if ( prop["use_limb_collision"] == "true" ) {
            m_use_limb_collision = true;
            std::cerr << "[" << m_profile.instance_name << "] Enable use_limb_collision" << std::endl;
        }
    }

    // setup collision state
    m_state.angle.length(m_robot->numJoints());
    m_state.collide.length(m_robot->numLinks());

    // allocate memory for outPorts
    m_q.data.length(m_robot->numJoints());
    m_recover_time = 0;
    m_safe_posture = true;
    m_have_safe_posture = false;
    i_dt = 1.0;
    default_recover_time = 2.5/m_dt;
    m_recover_jointdata = new double[m_robot->numJoints()];
    m_lastsafe_jointdata = new double[m_robot->numJoints()];
    m_interpolator = new interpolator(m_robot->numJoints(), i_dt);
    m_interpolator->setName(std::string(m_profile.instance_name)+" interpolator");
    m_link_collision = new bool[m_robot->numLinks()];

    for(int i=0; i<m_robot->numJoints(); i++){
      m_q.data[i] = 0;
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

    collision_beep_freq = static_cast<int>(1.0/(3.0*m_dt)); // 3 times / 1[s]
    m_beepCommand.data.length(bc.get_num_beep_info());
    return RTC::RTC_OK;
}



RTC::ReturnCode_t CollisionDetector::onFinalize()
{
    delete[] m_recover_jointdata;
    delete[] m_lastsafe_jointdata;
    delete m_interpolator;
    delete[] m_link_collision;
    return RTC::RTC_OK;
}

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
    std::cerr << "[" << m_profile.instance_name<< "] onActivated(" << ec_id << ")" << std::endl;
    m_have_safe_posture = false;
    return RTC::RTC_OK;
}

RTC::ReturnCode_t CollisionDetector::onDeactivated(RTC::UniqueId ec_id)
{
    std::cerr << "[" << m_profile.instance_name<< "] onDeactivated(" << ec_id << ")" << std::endl;
    return RTC::RTC_OK;
}

#define DEBUGP ((m_debugLevel==1 && loop%200==0) || m_debugLevel > 1 )
RTC::ReturnCode_t CollisionDetector::onExecute(RTC::UniqueId ec_id)
{
    static int loop = 0;
    loop++;

    // Connection check of m_beepCommand to BeeperRTC
    //   If m_beepCommand is not connected to BeeperRTC and sometimes, check connection.
    //   If once connection is found, never check connection.
    if (!is_beep_port_connected && (loop % 500 == 0) ) {
      if ( m_beepCommandOut.connectors().size() > 0 ) {
        is_beep_port_connected = true;
        std::cerr << "[" << m_profile.instance_name<< "] beepCommand data port connection found! Use BeeperRTC." << std::endl;
      }
    }

    if (m_servoStateIn.isNew()) {
        m_servoStateIn.read();
    }
    if ( ! m_enable ) {
        if ( DEBUGP || loop % 100 == 1) {
            std::cerr << "[" << m_profile.instance_name << "] CAUTION!! The robot is moving without checking self collision detection!!! please send enableCollisionDetection to CollisoinDetection RTC" << std::endl;
        }
        if ( m_qRefIn.isNew()) {
            m_qRefIn.read();
            for ( int i = 0; i < m_q.data.length(); i++ ) {
                m_q.data[i] = m_qRef.data[i];
            }
            m_q.tm = m_qRef.tm;
            m_qOut.write();
        }
    }
    if (m_enable && m_qRefIn.isNew()) {
	m_qRefIn.read();

        // check servo for collision beep sound
        bool has_servoOn = false;
        for (int i = 0; i < m_robot->numJoints(); i++ ){
          int servo_state = (m_servoState.data[i][0] & OpenHRP::RobotHardwareService::SERVO_STATE_MASK) >> OpenHRP::RobotHardwareService::SERVO_STATE_SHIFT;
          has_servoOn = has_servoOn || (servo_state == 1);
        }

        TimedPosture tp;

	assert(m_qRef.data.length() == m_robot->numJoints());
#ifdef USE_HRPSYSUTIL
        if ( m_use_viewer ) {
          for (int i=0; i<m_glbody->numLinks(); i++){
            ((GLlink *)m_glbody->link(i))->highlight(false);
          }
        }
        for (int i=0; i<m_glbody->numLinks(); i++){
            m_link_collision[m_glbody->link(i)->index] = false;
        }
#else
        for (int i=0; i<m_robot->numLinks(); i++){
            m_link_collision[m_robot->link(i)->index] = false;
        }
#endif // USE_HRPSYSUTIL

        //set robot model's angle for collision check(two types)
        //  1. current safe angle or collision angle  .. check based on qRef
        //  2. recovery                               .. check based on q'(m_recover_jointdata)
        if (m_recover_time == 0 || m_recover_time == default_recover_time ) {  // 1. current safe angle or collision angle
            if ( m_loop_for_check == 0 ) { // update robot posutre for each m_loop_for_check timing
                for ( int i = 0; i < m_robot->numJoints(); i++ ){
                    m_robot->joint(i)->q = m_qRef.data[i];
                }
            }
        }else{  // 2. recovery
          for ( int i = 0; i < m_robot->numJoints(); i++ ){
            if ( m_loop_for_check == 0 ) { // update robot posutre for each m_loop_for_check timing
              if ( m_curr_collision_mask[i] == 1) {// joint with 1 (do not move when collide :default), need to be updated using recover(safe) data
                  m_robot->joint(i)->q = m_recover_jointdata[i];
              }else{                               // joint with 0 (move even if collide), need to be updated using reference(dangerous) data
                  m_robot->joint(i)->q = m_qRef.data[i];
              }
            }
          }
        }
        //        }
        //collision check process in case of angle set above
	m_robot->calcForwardKinematics();
	coil::TimeValue tm1 = coil::gettimeofday();
        std::map<std::string, CollisionLinkPair *>::iterator it = m_pair.begin();
	for (unsigned int i = 0; it != m_pair.end(); it++, i++){
            int sub_size = (m_pair.size() + m_collision_loop -1) / m_collision_loop;  // 10 / 3 = 3  / floor
            // 0 : 0 .. sub_size-1                            // 0 .. 2
            // 1 : sub_size ... sub_size*2-1                  // 3 .. 5
            // k : sub_size*k ... sub_size*(k+1)-1            // 6 .. 8
            // n : sub_size*n ... m_pair.size()               // 9 .. 10
            if ( sub_size*m_loop_for_check <= i && i < sub_size*(m_loop_for_check+1) ) {
                CollisionLinkPair* c = it->second;
                c->distance = c->pair->computeDistance(c->point0.data(), c->point1.data());
                //std::cerr << i << ":" << (c->distance<=c->pair->getTolerance() ) << "/" << c->distance << " ";
            }
        }
        if ( m_loop_for_check == m_collision_loop-1 ) {
            bool last_safe_posture = m_safe_posture;
            m_safe_posture = true;
            it = m_pair.begin();
            for (unsigned int i = 0; it != m_pair.end(); i++, it++){
                CollisionLinkPair* c = it->second;
                VclipLinkPairPtr p = c->pair;
                tp.lines.push_back(std::make_pair(c->point0, c->point1));
                if ( c->distance <= c->pair->getTolerance() ) {
                    m_safe_posture = false;
                    if ( loop%200==0 || last_safe_posture ) {
                        hrp::JointPathPtr jointPath = m_robot->getJointPath(p->link(0),p->link(1));
                        std::cerr << "[" << m_profile.instance_name << "] " << i << "/" << m_pair.size() << " pair: " << p->link(0)->name << "/" << p->link(1)->name << "(" << jointPath->numJoints() << "), distance = " << c->distance << std::endl;
                    }
                    m_link_collision[p->link(0)->index] = true;
                    m_link_collision[p->link(1)->index] = true;
                    if ( m_use_limb_collision ) {
                        hrp::JointPathPtr jointPath = m_robot->getJointPath(p->link(0),p->link(1));
                        bool stop_all = true;
                        // if all joint is within false(0:move even if collide) in initial mask ( for example leg to leg ) we stop them
                        // if some joint is not within true(1:do not move within collide) on initial mask, stop only true joint (for exmple leg to arm)
                        for ( int i = 0; i < jointPath->numJoints(); i++ ){ if ( m_init_collision_mask[jointPath->joint(i)->jointId] == 1) stop_all = false; }
                        for ( int i = 0; i < jointPath->numJoints(); i++ ){
                            int id = jointPath->joint(i)->jointId;
                            // collision_mask used to select output                0: passthough reference data, 1 output safe data
                            if ( stop_all ) {
                                m_curr_collision_mask[id] = 1;            // true (1: output safe data) do not move when collide,
                            } else if (m_init_collision_mask[id] == 1) {  // skip false (0: move even if collide)
                                m_curr_collision_mask[id] = 1;
                            }
                        }
                    } else {
                        std::copy(m_init_collision_mask.begin(), m_init_collision_mask.end(), m_curr_collision_mask.begin()); // copy init_collision_mask to curr_collision_mask
                    }
#ifdef USE_HRPSYSUTIL
                    if ( m_use_viewer ) {
                        ((GLlink *)p->link(0))->highlight(true);
                        ((GLlink *)p->link(1))->highlight(true);
                    }
#endif // USE_HRPSYSUTIL
                }
            }
            if ( m_safe_posture ) {
                if (has_servoOn) {
                if (! m_have_safe_posture ) {
                    // first transition collision -> safe
                    std::cerr << "[" << m_profile.instance_name << "] set safe posture" << std::endl;
                    for ( int i = 0; i < m_q.data.length(); i++ ) {
                        m_lastsafe_jointdata[i] = m_robot->joint(i)->q;
                    }
                    m_interpolator->set(m_lastsafe_jointdata); // Set current angles as initial angle for recover
                }
                m_have_safe_posture = true;
                }
                if (m_recover_time != default_recover_time) {
                    // sefe or recover
                    // in collision, robot->q may differ from m_q.
                    for ( int i = 0; i < m_q.data.length(); i++ ) {
                        m_lastsafe_jointdata[i] = m_robot->joint(i)->q;
                    }
                }
            }else{
                for ( int i = 0; i < m_q.data.length(); i++ ) {
                    if ( m_curr_collision_mask[i] == 0 ) { // if collisoin_mask is 0 (move even if collide), we update lastsafe_joint_data from input data
                        m_lastsafe_jointdata[i] = m_robot->joint(i)->q;
                    }
                }
            }
            if ( m_use_limb_collision ) {
               if ( loop%200==0 and ! m_safe_posture ) {
                   std::cerr << "[" << m_profile.instance_name << "] collision_mask : ";
                    for (size_t i = 0; i < m_robot->numJoints(); i++) {
                        std::cerr << m_robot->joint(i)->name << ":"  << m_curr_collision_mask[i] << " ";
                    }
                    std::cerr << std::endl;
                }
            }
        }
        //     mode : m_safe_posture : recover_time  : set as q
        // safe     :           true :            0  : qRef
        // collison :          false :         >  0  : q( do nothing)
        // recover  :           true :         >  0  : q'
        //std::cerr << "m_recover_time: " << m_recover_time << std::endl;
        coil::TimeValue tm2 = coil::gettimeofday();
        if (m_safe_posture && m_recover_time == 0){ // safe mode
          //std::cerr << "safe-------------- " << std::endl;
          for ( int i = 0; i < m_q.data.length(); i++ ) {
            m_q.data[i] = m_qRef.data[i];
          }
          // collision_mask used to select output                0: passthough reference data, 1 output safe data
          std::fill(m_curr_collision_mask.begin(), m_curr_collision_mask.end(), 0); // false(0) clear output data
        } else {
          static int collision_loop_recover = 0;
          if(m_safe_posture){  //recover
            //std::cerr << "recover-------------- " << std::endl;
            for ( int i = 0; i < m_q.data.length(); i++ ) {
              m_q.data[i] = m_recover_jointdata[i];
            }
            m_recover_time = m_recover_time - i_dt;
          } else if (m_collision_loop > 1 && (m_recover_time != default_recover_time)) { // collision with collision_loop
            //std::cerr << "collision_loop-------------- " << std::endl;
            collision_loop_recover = m_collision_loop;
            m_recover_time = default_recover_time;      // m_recover_time should be set based on difference between qRef and q
          } else { //collision
            //std::cerr << "collision-------------- " << std::endl;
            //do nothing (stay previous m_q)
            m_recover_time = default_recover_time;      // m_recover_time should be set based on difference between qRef and q
            m_interpolator->set(m_lastsafe_jointdata); //Set last safe joint data as initial angle
            //m_interpolator->set(m_q.data.get_buffer()); //Set initial angle
          }
          for ( int i = 0; i < m_q.data.length(); i++ ) {
              if (m_curr_collision_mask[i] == 0) { // 0: passthough reference data, 1 output safe data, stop joints only joint with 1
                  m_q.data[i] = m_qRef.data[i];
              }
          }
          //calc q'
#if 0
          //linear interpolation (dangerous)
          for ( int i = 0; i < m_q.data.length(); i++ ) {
            m_recover_jointdata[i] = m_q.data[i] + (m_qRef.data[i] - m_q.data[i]) / m_recover_time;
          }
#else
          if (collision_loop_recover != 0) {
            collision_loop_recover--;
            for ( int i = 0; i < m_q.data.length(); i++ ) {
              m_q.data[i] =
                  m_lastsafe_jointdata[i] + collision_loop_recover * ((m_q.data[i] - m_lastsafe_jointdata[i])/m_collision_loop);
            }
          } else {
          collision_loop_recover = 0;
          //minjerk interpolation
          m_interpolator->setGoal(m_qRef.data.get_buffer(), m_recover_time);
          m_interpolator->get(m_recover_jointdata);
          }
#endif
        }
        if ( DEBUGP ) {
          std::cerr << "[" << m_profile.instance_name << "] check collisions for " << m_pair.size() << " pairs in " << (tm2.sec()-tm1.sec())*1000+(tm2.usec()-tm1.usec())/1000.0 
                    << " [msec], safe = " << m_safe_posture << ", time = " << m_recover_time*m_dt << "[s], loop = " << m_loop_for_check << "/" << m_collision_loop << std::endl;
        }
        if ( m_pair.size() == 0 && ( DEBUGP || (loop % ((int)(5/m_dt))) == 1) ) {
            std::cerr << "[" << m_profile.instance_name << "] CAUTION!! The robot is moving without checking self collision detection!!! please define collision_pair in configuration file" << std::endl;
        }
        if ( ! m_have_safe_posture && ! m_safe_posture ) {
            if ( DEBUGP || (loop % ((int)(5/m_dt))) == 1) {
                std::cerr << "[" << m_profile.instance_name << "] CAUTION!! The robot is moving while collision detection!!!, since we do not get safe_posture yet" << std::endl;
            }
            for ( int i = 0; i < m_q.data.length(); i++ ) {
                m_lastsafe_jointdata[i] = m_recover_jointdata[i] = m_q.data[i] = m_qRef.data[i];
            }
        }
        //
        m_q.tm = m_qRef.tm;
        m_qOut.write();

        // if servo off, we do not know last safe posture
        if (! has_servoOn ) {
            m_have_safe_posture = false;
        }
        // beep sound for collision alert
        if ( !m_safe_posture && has_servoOn ) { // If collided and some joint is servoOn
          if (is_beep_port_connected) {
            if ( collision_beep_count % collision_beep_freq == 0 && collision_beep_count % (collision_beep_freq * 3) != 0 ) bc.startBeep(2352, collision_beep_freq*0.7);
            else bc.stopBeep();
          } else {
            if ( collision_beep_count % collision_beep_freq == 0 && collision_beep_count % (collision_beep_freq * 3) != 0 ) start_beep(2352, collision_beep_freq*0.7);
            else stop_beep();
          }
          collision_beep_count++;
        } else {
          if (is_beep_port_connected) bc.stopBeep();
          collision_beep_count = 0;
        }

        if ( ++m_loop_for_check >= m_collision_loop ) m_loop_for_check = 0;
        tp.posture.resize(m_qRef.data.length());
        for (size_t i=0; i<tp.posture.size(); i++) tp.posture[i] = m_q.data[i];
#ifdef USE_HRPSYSUTIL
        m_log.add(tp);
#endif // USE_HRPSYSUTIL

        // set collisoin state
        m_state.time = tm2;
        for (int i = 0; i < m_robot->numJoints(); i++ ){
            m_state.angle[i] = m_robot->joint(i)->q;
        }

        if ( m_loop_for_check == 0 ) {
            for (int i = 0; i < m_robot->numLinks(); i++ ){
                m_state.collide[i] = m_link_collision[i];
            }

            m_state.lines.length(tp.lines.size());
            for(int i = 0; i < tp.lines.size(); i++ ){
                const std::pair<hrp::Vector3, hrp::Vector3>& line = tp.lines[i];
                double *v;
                m_state.lines[i].length(2);
                m_state.lines[i].get_buffer()[0].length(3);
                v = m_state.lines[i].get_buffer()[0].get_buffer();
                v[0] = line.first.data()[0];
                v[1] = line.first.data()[1];
                v[2] = line.first.data()[2];
                m_state.lines[i].get_buffer()[1].length(3);
                v = m_state.lines[i].get_buffer()[1].get_buffer();
                v[0] = line.second.data()[0];
                v[1] = line.second.data()[1];
                v[2] = line.second.data()[2];
            }
        }
         m_state.computation_time = (tm2-tm1)*1000.0;
        m_state.safe_posture = m_safe_posture;
        m_state.recover_time = m_recover_time;
        m_state.loop_for_check = m_loop_for_check;
    }
    if (is_beep_port_connected) {
      bc.setDataPort(m_beepCommand);
      if (bc.isWritable()) m_beepCommandOut.write();
    }
#ifdef USE_HRPSYSUTIL
    if ( m_use_viewer ) m_window.oneStep();
#endif // USE_HRPSYSUTIL
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

bool CollisionDetector::setTolerance(const char *i_link_pair_name, double i_tolerance) {
    if (strcmp(i_link_pair_name, "all") == 0 || strcmp(i_link_pair_name, "ALL") == 0){
        for ( std::map<std::string, CollisionLinkPair *>::iterator it = m_pair.begin();  it != m_pair.end(); it++){
            it->second->pair->setTolerance(i_tolerance);
        }
    }else if ( m_pair.find(std::string(i_link_pair_name)) != m_pair.end() ) {
        m_pair[std::string(i_link_pair_name)]->pair->setTolerance(i_tolerance);
    }else{
        return false;
    }
    return true;
}

bool CollisionDetector::getCollisionStatus(OpenHRP::CollisionDetectorService::CollisionState &state)
{
    state = m_state;
    return true;
}

void CollisionDetector::setupVClipModel(hrp::BodyPtr i_body)
{
    m_VclipLinks.resize(i_body->numLinks());
    //std::cerr << i_body->numLinks() << std::endl;
    for (int i=0; i<i_body->numLinks(); i++) {
      assert(i_body->link(i)->index == i);
      setupVClipModel(i_body->link(i));
    }
}

bool CollisionDetector::checkIsSafeTransition(void)
{
    for ( int i = 0; i < m_q.data.length(); i++ ) {
        // If servoOn, check too large joint angle gap. Otherwise (servoOff), neglect too large joint angle gap.
        int servo_state = (m_servoState.data[i][0] & OpenHRP::RobotHardwareService::SERVO_STATE_MASK) >> OpenHRP::RobotHardwareService::SERVO_STATE_SHIFT; // enum SwitchStatus {SWITCH_ON, SWITCH_OFF};
        if (servo_state == 1 && abs(m_q.data[i] - m_qRef.data[i]) > 0.017) return false;
    }
    return true;
}

bool CollisionDetector::enable(void)
{
    if (m_enable){
        std::cerr << "[" << m_profile.instance_name << "] CollisionDetector is already enabled." << std::endl;
        return true;
    }

    if (!checkIsSafeTransition()){
        std::cerr << "[" << m_profile.instance_name << "] CollisionDetector cannot be enabled because of different reference joint angle" << std::endl;
        return false;
    }

    // check collision
    for ( int i = 0; i < m_robot->numJoints(); i++ ){
        m_robot->joint(i)->q = m_qRef.data[i];
    }
    m_robot->calcForwardKinematics();
    std::map<std::string, CollisionLinkPair *>::iterator it = m_pair.begin();
    for (unsigned int i = 0; it != m_pair.end(); it++, i++){
        CollisionLinkPair* c = it->second;
        VclipLinkPairPtr p = c->pair;
        c->distance = c->pair->computeDistance(c->point0.data(), c->point1.data());
        if ( c->distance <= c->pair->getTolerance() ) {
            hrp::JointPathPtr jointPath = m_robot->getJointPath(p->link(0),p->link(1));
            std::cerr << "[" << m_profile.instance_name << "] CollisionDetector cannot be enabled because of collision" << std::endl;
            std::cerr << "[" << m_profile.instance_name << "] " << i << "/" << m_pair.size() << " pair: " << p->link(0)->name << "/" << p->link(1)->name << "(" << jointPath->numJoints() << "), distance = " << c->distance << std::endl;
            return false;
        }
    }
    std::cerr << "[" << m_profile.instance_name << "] CollisionDetector is successfully enabled." << std::endl;

    m_safe_posture = true;
    m_recover_time = 0;
    m_loop_for_check = 0;
    m_enable = true;
    return true;
}

bool CollisionDetector::disable(void)
{
    if (!checkIsSafeTransition()){
        std::cerr << "[" << m_profile.instance_name << "] CollisionDetector cannot be disabled because of different reference joint angle" << std::endl;
        return false;
    }
    std::cerr << "[" << m_profile.instance_name << "] CollisionDetector is successfully disabled." << std::endl;
    m_enable = false;
    return true;
}

void CollisionDetector::setupVClipModel(hrp::Link *i_link)
{
    Vclip::Polyhedron* i_vclip_model = new Vclip::Polyhedron();
    int n = i_link->coldetModel->getNumVertices();
    float v[3];
    Vclip::VertFaceName vertName;
    for (int i = 0; i < n; i ++ ) {
        i_link->coldetModel->getVertex(i, v[0], v[1], v[2]);
        sprintf(vertName, "v%d", i);
        i_vclip_model->addVertex(vertName, Vclip::Vect3(v[0], v[1], v[2]));
    }
    i_vclip_model->buildHull();
    i_vclip_model->check();
    m_VclipLinks[i_link->index] = i_vclip_model;
}

#ifndef USE_HRPSYSUTIL
hrp::Link *hrplinkFactory()
{
  return new hrp::Link();
}
#endif // USE_HRPSYSUTIL

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



#include "robot.h"
#include "hrpsys/util/Hrpsys.h"
#include <iostream>

#define DEFAULT_ANGLE_ERROR_LIMIT (0.2 - 0.02) // [rad]


// robot model copy from RobotHardware
robot::robot() {
}
robot::~robot() {
}
bool robot::init() {
  m_servoErrorLimit.resize(numJoints());
  for (unsigned int i=0; i<numJoints(); i++){
    m_servoErrorLimit[i] = DEFAULT_ANGLE_ERROR_LIMIT;
  }
  return true;
}

bool robot::setServoErrorLimit(const char *i_jname, double i_limit) {
  hrp::Link *l = NULL;
  if (strcmp(i_jname, "all") == 0 || strcmp(i_jname, "ALL") == 0){
    for (unsigned int i=0; i<numJoints(); i++){
      m_servoErrorLimit[i] = i_limit;
    }
    std::cerr << "[el] setServoErrorLimit " << i_limit << "[rad] for all joints" << std::endl;
  }else if ((l = link(i_jname))){
    m_servoErrorLimit[l->jointId] = i_limit;
    std::cerr << "[el] setServoErrorLimit " << i_limit << "[rad] for " << i_jname << std::endl;
  }else{
    std::cerr << "[el] Invalid joint name of setServoErrorLimit " << i_jname << "!" << std::endl;
    return false;
  }
  return true;
}

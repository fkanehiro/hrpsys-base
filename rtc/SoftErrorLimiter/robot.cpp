#include "robot.h"

#define DEFAULT_ANGLE_ERROR_LIMIT (0.2 - 0.02) // [rad]

// robot model copy from RobotHardware
robot::robot() {
}
robot::~robot() {
}
bool robot::init() {
  m_servoErrorLimit.resize(numJoints());
  for (int i=0; i<numJoints(); i++){
    m_servoErrorLimit[i] = DEFAULT_ANGLE_ERROR_LIMIT;
  }
  return true;
}

bool robot::setServoErrorLimit(const char *i_jname, double i_limit) {
  hrp::Link *l = NULL;
  if (strcmp(i_jname, "all") == 0 || strcmp(i_jname, "ALL") == 0){
    for (int i=0; i<numJoints(); i++){
      m_servoErrorLimit[i] = i_limit;
    }
  }else if ((l = link(i_jname))){
    m_servoErrorLimit[l->jointId] = i_limit;
  }else{
    return false;
  }
  return true;
}

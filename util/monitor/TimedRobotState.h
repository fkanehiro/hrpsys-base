#ifndef __TIMED_ROBOT_STATE_H__
#define __TIMED_ROBOT_STATE_H__

#include "RobotHardwareService.hh"

typedef struct
{
    double time;
    OpenHRP::RobotHardwareService::RobotState state;
} TimedRobotState;

#endif


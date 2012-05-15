#ifndef __TIMED_ROBOT_STATE_H__
#define __TIMED_ROBOT_STATE_H__

#include "RobotHardwareService.hh"
#include "StateHolderService.hh"

typedef struct
{
    double time;
    OpenHRP::StateHolderService::Command command;
    OpenHRP::RobotHardwareService::RobotState state;
} TimedRobotState;

#endif


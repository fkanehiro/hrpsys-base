#ifndef __TIMED_ROBOT_STATE_H__
#define __TIMED_ROBOT_STATE_H__

#include "hrpsys/idl/RobotHardwareService.hh"
#include "hrpsys/idl/StateHolderService.hh"

typedef struct {
  double time;
  OpenHRP::StateHolderService::Command command;
  OpenHRP::RobotHardwareService::RobotState state;
} TimedRobotState;

#endif

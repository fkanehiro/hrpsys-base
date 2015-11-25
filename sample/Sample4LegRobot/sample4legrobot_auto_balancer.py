#!/usr/bin/env python

try:
    from hrpsys.hrpsys_config import *
    import OpenHRP
except:
    print "import without hrpsys"
    import rtm
    from rtm import *
    from OpenHRP import *
    import waitInput
    from waitInput import *
    import socket
    import time

import math

def init ():
    global hcf, initial_pose, hrpsys_version
    hcf = HrpsysConfigurator()
    hcf.getRTCList = hcf.getRTCListUnstable
    hcf.init ("Sample4LegRobot(Robot)0", "$(PROJECT_DIR)/../model/sample_4leg_robot.wrl")
    initial_pose = [0,  -0.378613,  0,  0.832038,  -0.452564,  0,
                    0,  -0.378613,  0,  0.832038,  -0.452564,  0,
                    0,  -0.378613,  0,  0.832038,  -0.452564,  0,
                    0,  -0.378613,  0,  0.832038,  -0.452564,  0,]
    hcf.seq_svc.setJointAngles(initial_pose, 2.0)
    hcf.waitInterpolation()
    # set abc param
    abcp=hcf.abc_svc.getAutoBalancerParam()[1]
    abcp.leg_names = ['rleg', 'lleg', 'rarm', 'larm']
    hcf.abc_svc.setAutoBalancerParam(abcp)
    # set gg param
    ggp = hcf.abc_svc.getGaitGeneratorParam()[1]
    ggp.zmp_weight_map = [1.0]*4
    ggp.default_step_height = 0.065 # see https://github.com/fkanehiro/hrpsys-base/issues/801
    hcf.abc_svc.setGaitGeneratorParam(ggp)
    hcf.startAutoBalancer(limbs=['rleg','lleg','rarm','larm'])
    hrpsys_version = hcf.seq.ref.get_component_profile().version
    print("hrpsys_version = %s"%hrpsys_version)

def demoGaitGeneratorSetFootSteps(print_str="1. setFootSteps"):
    print >> sys.stderr, print_str
    hcf.setFootSteps([OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0.0+0.00,-0.19,0], [1,0,0,0], "rleg"),
                                                             OpenHRP.AutoBalancerService.Footstep([0.7+0.00,+0.19,0], [1,0,0,0], "larm")]),
                      OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0.0+0.15,+0.19,0], [1,0,0,0], "lleg"),
                                                             OpenHRP.AutoBalancerService.Footstep([0.7+0.15,-0.19,0], [1,0,0,0], "rarm")]),
                      OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0.0+0.30,-0.19,0], [1,0,0,0], "rleg"),
                                                             OpenHRP.AutoBalancerService.Footstep([0.7+0.30,+0.19,0], [1,0,0,0], "larm")]),
                      OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0.0+0.45,+0.19,0], [1,0,0,0], "lleg"),
                                                             OpenHRP.AutoBalancerService.Footstep([0.7+0.45,-0.19,0], [1,0,0,0], "rarm")]),
                      OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0.0+0.45,-0.19,0], [1,0,0,0], "rleg"),
                                                             OpenHRP.AutoBalancerService.Footstep([0.7+0.45,+0.19,0], [1,0,0,0], "larm")])])
    hcf.abc_svc.waitFootSteps()

def demoGaitGeneratorSetFootStepsRectangle():
    ggp=hcf.abc_svc.getGaitGeneratorParam()[1];
    ggp.swing_trajectory_delay_time_offset=0.05;
    ggp.default_orbit_type=OpenHRP.AutoBalancerService.RECTANGLE;
    hcf.abc_svc.setGaitGeneratorParam(ggp)
    demoGaitGeneratorSetFootSteps("2. setFootSteps with Rectangle orbit");

def demoGaitGeneratorSetFootStepsCycloidDelay():
    ggp=hcf.abc_svc.getGaitGeneratorParam()[1];
    ggp.swing_trajectory_delay_time_offset=0.05;
    ggp.swing_trajectory_final_distance_weight=3.0;
    ggp.default_orbit_type=OpenHRP.AutoBalancerService.CYCLOIDDELAY;
    hcf.abc_svc.setGaitGeneratorParam(ggp)
    demoGaitGeneratorSetFootSteps("3. setFootSteps with Cycloiddelay orbit");

def demo():
    init()
    demoGaitGeneratorSetFootSteps()
    demoGaitGeneratorSetFootStepsRectangle()
    demoGaitGeneratorSetFootStepsCycloidDelay()

if __name__ == '__main__':
    demo()

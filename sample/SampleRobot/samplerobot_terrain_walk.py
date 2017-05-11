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

def init ():
    global hcf
    hcf = HrpsysConfigurator()
    hcf.getRTCList = hcf.getRTCListUnstable
    hcf.init ("SampleRobot(Robot)0", "$(PROJECT_DIR)/../model/sample1.wrl")

def initPose():
    # set initial pose from base 90mm down pose of sample/controller/SampleController/etc/Sample.pos
    initial_pose=[-0.000181,-0.614916,-0.000239,1.36542,-0.749643,0.000288,0.31129,-0.159481,-0.115399,-0.636277,0.0,0.0,0.0,-0.000181,-0.614916,-0.000239,1.36542,-0.749643,0.000288,0.31129,0.159481,0.115399,-0.636277,0.0,0.0,0.0,0.0,0.0,0.0]
    hcf.seq_svc.setJointAngles(initial_pose, 1.0)
    hcf.seq_svc.waitInterpolation()
    hcf.startAutoBalancer();

def demo():
    init()
    initPose()

def setupGaitGeneratorParam(set_step_height=False):
    ggp = hcf.abc_svc.getGaitGeneratorParam()
    ggp[1].default_double_support_ratio = 0.3
    ggp[1].default_step_time = 1.2
    if set_step_height:
        ggp[1].default_step_height = 0.1
    #ggp[1].swing_trajectory_delay_time_offset = 0.2;
    ggp[1].default_orbit_type = OpenHRP.AutoBalancerService.STAIR;
    hcf.abc_svc.setGaitGeneratorParam(ggp[1])

def stairWalk(stair_height = 0.1524):
    stair_stride_x = 0.25
    floor_stride_x = 0.16
    init_step_x = 0
    init_step_z = 0
    ret = []
    setupGaitGeneratorParam()
    for step_idx in [1,2,3,4]:
        ret = ret + [OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([init_step_x, -0.09, init_step_z], [1,0,0,0], "rleg")]),
                     OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([init_step_x+stair_stride_x, 0.09, init_step_z+stair_height], [1,0,0,0], "lleg")]),
                     OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([init_step_x+stair_stride_x, -0.09, init_step_z+stair_height], [1,0,0,0], "rleg")]),
                     OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([init_step_x+stair_stride_x+floor_stride_x, 0.09, init_step_z+stair_height], [1,0,0,0], "lleg")])]
        init_step_x = init_step_x + stair_stride_x + floor_stride_x
        init_step_z = init_step_z + stair_height
    hcf.setFootSteps(ret)
    hcf.abc_svc.waitFootSteps()

# sample for SampleRobot.TerrainFloor.SlopeUpDown.xml
def demoSlopeUpDown():
    print "Start stlop up down"
    setupGaitGeneratorParam(True)
    hcf.abc_svc.startAutoBalancer(["rleg", "lleg"]);
    fsList=[OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0.8,-0.09,0.0], [1.0,0.0,2.775558e-17,0.0], "rleg")]),
            OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([1.0953,0.09,0.030712], [0.991445,0.0,-0.130526,0.0], "lleg")]),
            OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([1.28848,-0.09,0.082475], [0.991445,0.0,-0.130526,0.0], "rleg")]),
            OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([1.38508,0.09,0.108357], [0.991445,0.0,-0.130526,0.0], "lleg")]),
            OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([1.38508,-0.09,0.108357], [0.991445,0.0,-0.130526,0.0], "rleg")]),
            OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([1.54959,0.09,0.125863], [0.991445,0.0,0.130526,0.0], "lleg")]),
            OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([1.74277,-0.09,0.074099], [0.991445,0.0,0.130526,0.0], "rleg")]),
            OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([1.79107,0.09,0.061158], [0.991445,0.0,0.130526,0.0], "lleg")]),
            OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([2.05,-0.09,0.0], [1.0,0.0,0.0,0.0], "rleg")]),
            OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([2.05,0.09,0.0], [1.0,0.0,0.0,0.0], "lleg")])]
    # set st Parameter
    # stp1 = hcf.st_svc.getParameter()
    # stp1.k_tpcc_p=[0.05, 0.05]
    # stp1.k_tpcc_x=[4.0, 4.0]
    # stp1.k_brot_p=[0.0, 0.0]
    # hcf.st_svc.setParameter(stp1)
    # hcf.st_svc.startStabilizer ()
    for idx in range(len(fsList)-1):
        hcf.setFootSteps([fsList[idx],fsList[idx+1]])
        hcf.abc_svc.waitFootSteps()
    hcf.abc_svc.stopAutoBalancer();

# sample for SampleRobot.TerrainFloor.StairUp.xml
def demoStairUp():
    print "Start stair up"
    stairWalk()

# sample for SampleRobot.TerrainFloor.StairDown.xml
def demoStairDown():
    print "Start stair down"
    hcf.abc_svc.goPos(0.05, 0.0, 0.0)
    hcf.abc_svc.waitFootSteps()
    stairWalk(-0.1524)

def demoStairUpDown():
    print "Start stair up"
    stairWalk()
    hcf.abc_svc.goPos(0.05, 0.0, 0.0)
    hcf.abc_svc.waitFootSteps()
    print "Start stair down"
    stairWalk(-0.1524)

if __name__ == '__main__':
    demo()

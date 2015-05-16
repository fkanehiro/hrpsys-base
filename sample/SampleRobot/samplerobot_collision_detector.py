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
    global hcf, init_pose, col_safe_pose, col_fail_pose
    hcf = HrpsysConfigurator()
    hcf.getRTCList = hcf.getRTCListUnstable
    hcf.init ("SampleRobot(Robot)0", "$(PROJECT_DIR)/../model/sample1.wrl")
    init_pose = [0]*29
    col_safe_pose = [0.0,-0.349066,0.0,0.820305,-0.471239,0.0,0.523599,0.0,0.0,-1.74533,0.15708,-0.113446,0.0,0.0,-0.349066,0.0,0.820305,-0.471239,0.0,0.523599,0.0,0.0,-1.74533,-0.15708,-0.113446,0.0,0.0,0.0,0.0]
    col_fail_pose = [0.0,-0.349066,0.0,0.820305,-0.471239,0.0,0.845363,0.03992,0.250074,-1.32816,0.167513,0.016204,0.0,0.0,-0.349066,0.0,0.820305,-0.471239,0.0,0.523599,0.0,0.0,-1.74533,-0.15708,-0.113446,0.0,0.0,0.0,0.0]

# demo functions
def demoCollisionCheckSafe ():
    print "1. CollisionCheck in safe pose"
    hcf.seq_svc.setJointAngles(col_safe_pose, 1.0);
    hcf.seq_svc.waitInterpolation();
    cs=hcf.co_svc.getCollisionStatus()[1]
    if cs.safe_posture:
        print "=> Safe pose"

def demoCollisionCheckFail ():
    print "2. CollisionCheck in fail pose"
    hcf.seq_svc.setJointAngles(col_fail_pose, 1.0);
    hcf.seq_svc.waitInterpolation();
    cs=hcf.co_svc.getCollisionStatus()[1]
    if not cs.safe_posture:
        print "=> Successfully stop fail pose"
    hcf.seq_svc.setJointAngles(col_safe_pose, 3.0);
    hcf.seq_svc.waitInterpolation();
    cs=hcf.co_svc.getCollisionStatus()[1]
    if cs.safe_posture:
        print "=> Successfully return to safe pose"

def demoCollisionCheckFailWithSetTolerance ():
    print "3. CollisionCheck in fail pose with 0.1[m] tolerance"
    hcf.co_svc.setTolerance("all", 0.1); # [m]
    hcf.seq_svc.setJointAngles(col_fail_pose, 1.0);
    hcf.seq_svc.waitInterpolation();
    cs=hcf.co_svc.getCollisionStatus()[1]
    if not cs.safe_posture:
        print "=> Successfully stop fail pose (0.1[m] tolerance)"
    hcf.seq_svc.setJointAngles(col_safe_pose, 3.0);
    hcf.seq_svc.waitInterpolation();
    cs=hcf.co_svc.getCollisionStatus()[1]
    if cs.safe_posture:
        print "=> Successfully return to safe pose"

def demoCollisionDisableEnable ():
    print "4. CollisionDetection enable and disable"
    hcf.seq_svc.setJointAngles(col_safe_pose, 1.0);
    hcf.seq_svc.waitInterpolation();
    if hcf.co_svc.disableCollisionDetection():
        print "=> Successfully disabled when no collision"
    if hcf.co_svc.enableCollisionDetection():
        print "=> Successfully enabled when no collision"
    hcf.seq_svc.setJointAngles(col_fail_pose, 1.0);
    hcf.seq_svc.waitInterpolation();
    if not hcf.co_svc.disableCollisionDetection():
        print "=> Successfully inhibit disabling when collision"
    hcf.seq_svc.setJointAngles(col_safe_pose, 1.0);
    hcf.seq_svc.waitInterpolation();

def demoCollisionMask ():
    if hcf.abc_svc != None:
        print "5. Collision mask test"
        hcf.co_svc.setTolerance("all", 0); # [m]
        hcf.startAutoBalancer()
        print "  5.1 Collision mask among legs : Check RLEG_ANKLE_R - LLEG_ANKLE_R"
        print "      Desired behavior : Robot stops when legs collision."
        hcf.abc_svc.setFootSteps([OpenHRP.AutoBalancerService.Footstep([0,-0.09,0],[1,0,0,0],"rleg"),OpenHRP.AutoBalancerService.Footstep([0,0.0,0],[1,0,0,0],"lleg")])
        hcf.abc_svc.waitFootSteps();
        hcf.abc_svc.setFootSteps([OpenHRP.AutoBalancerService.Footstep([0,-0.09,0],[1,0,0,0],"rleg"),OpenHRP.AutoBalancerService.Footstep([0,0.09,0],[1,0,0,0],"lleg")])
        hcf.abc_svc.waitFootSteps();
        print "  => Successfully mask works. Legs joints stops when collision."
        print "  5.2 Collision mask between leg and arm : Check RLEG_HIP_R and RARM_WRIST*"
        print "      Desired behavior : Leg joints moves and arm joints stops when collision."
        hcf.seq_svc.setJointAngles(col_safe_pose, 1.0);
        hcf.seq_svc.waitInterpolation();
        hcf.abc_svc.goVelocity(0,0,0);
        hcf.seq_svc.setJointAngles(col_fail_pose, 1.0);
        hcf.seq_svc.waitInterpolation();
        hcf.seq_svc.setJointAngles(col_safe_pose, 3.0);
        hcf.seq_svc.waitInterpolation();
        hcf.abc_svc.goStop();
        print "  => Successfully mask works. Arm joints stops and leg joints moves."
        print "  5.3 Collision mask between leg and arm : Check RLEG_HIP_R and RARM_WRIST* and RLEG_ANKLE_R and LLEG_ANKLE_R (combination of 5.1 and 5.2)"
        print "      Desired behavior : First, arm stops and legs moves."
        hcf.seq_svc.setJointAngles(col_safe_pose, 1.0);
        hcf.seq_svc.waitInterpolation();
        print "      Desired behavior : Next, arm keeps stopping and legs stops."
        hcf.abc_svc.setFootSteps([OpenHRP.AutoBalancerService.Footstep([0,-0.09,0],[1,0,0,0],"rleg"),OpenHRP.AutoBalancerService.Footstep([0,0.0,0],[1,0,0,0],"lleg")])
        hcf.abc_svc.waitFootSteps();
        hcf.abc_svc.setFootSteps([OpenHRP.AutoBalancerService.Footstep([0,-0.09,0],[1,0,0,0],"rleg"),OpenHRP.AutoBalancerService.Footstep([0,0.09,0],[1,0,0,0],"lleg")])
        hcf.abc_svc.waitFootSteps();
        print "  => Successfully mask works with combined situation."
        hcf.stopAutoBalancer()

def demo():
    init()
    demoCollisionCheckSafe()
    demoCollisionCheckFail()
    demoCollisionCheckFailWithSetTolerance()
    demoCollisionDisableEnable()
    demoCollisionMask()

if __name__ == '__main__':
    demo()

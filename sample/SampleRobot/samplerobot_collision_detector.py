#!/usr/bin/env python

from hrpsys.hrpsys_config import *
from hrpsys import OpenHRP

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
    print >> sys.stderr, "1. CollisionCheck in safe pose"
    hcf.seq_svc.setJointAngles(col_safe_pose, 1.0);
    hcf.waitInterpolation();
    cs=hcf.co_svc.getCollisionStatus()[1]
    if cs.safe_posture:
        print >> sys.stderr, "  => Safe pose"
    assert(cs.safe_posture is True)

def demoCollisionCheckFail ():
    print >> sys.stderr, "2. CollisionCheck in fail pose"
    hcf.seq_svc.setJointAngles(col_fail_pose, 1.0);
    hcf.waitInterpolation();
    cs=hcf.co_svc.getCollisionStatus()[1]
    if not cs.safe_posture:
        print >> sys.stderr, "  => Successfully stop fail pose"
    assert((not cs.safe_posture) is True)
    hcf.seq_svc.setJointAngles(col_safe_pose, 3.0);
    hcf.waitInterpolation();
    cs=hcf.co_svc.getCollisionStatus()[1]
    if cs.safe_posture:
        print >> sys.stderr, "  => Successfully return to safe pose"
    assert(cs.safe_posture is True)

def demoCollisionCheckFailWithSetTolerance ():
    print >> sys.stderr, "3. CollisionCheck in fail pose with 0.1[m] tolerance"
    hcf.co_svc.setTolerance("all", 0.1); # [m]
    hcf.seq_svc.setJointAngles(col_fail_pose, 1.0);
    hcf.waitInterpolation();
    cs=hcf.co_svc.getCollisionStatus()[1]
    if not cs.safe_posture:
        print >> sys.stderr, "  => Successfully stop fail pose (0.1[m] tolerance)"
    assert((not cs.safe_posture) is True)
    hcf.co_svc.setTolerance("all", 0.0); # [m]
    hcf.seq_svc.setJointAngles(col_safe_pose, 3.0);
    hcf.waitInterpolation();
    cs=hcf.co_svc.getCollisionStatus()[1]
    if cs.safe_posture:
        print >> sys.stderr, "  => Successfully return to safe pose"
    assert(cs.safe_posture is True)

def demoCollisionDisableEnable ():
    print >> sys.stderr, "4. CollisionDetection enable and disable"
    hcf.seq_svc.setJointAngles(col_safe_pose, 1.0);
    hcf.waitInterpolation();
    if hcf.co_svc.disableCollisionDetection():
        print >> sys.stderr, "  => Successfully disabled when no collision"
    assert(hcf.co_svc.disableCollisionDetection() is True)
    if hcf.co_svc.enableCollisionDetection():
        print >> sys.stderr, "  => Successfully enabled when no collision"
    assert(hcf.co_svc.enableCollisionDetection() is True)
    hcf.seq_svc.setJointAngles(col_fail_pose, 1.0);
    hcf.waitInterpolation();
    if not hcf.co_svc.disableCollisionDetection():
        print >> sys.stderr, "  => Successfully inhibit disabling when collision"
    assert((not hcf.co_svc.disableCollisionDetection()) is True)
    hcf.seq_svc.setJointAngles(col_safe_pose, 1.0);
    hcf.waitInterpolation();

def demoCollisionMask ():
    if hcf.abc_svc != None:
        print >> sys.stderr, "5. Collision mask test"
        hcf.co_svc.setTolerance("all", 0); # [m]
        hcf.startAutoBalancer()
        print >> sys.stderr, "  5.1 Collision mask among legs : Check RLEG_ANKLE_R - LLEG_ANKLE_R"
        print >> sys.stderr, "      Desired behavior : Robot stops when legs collision."
        hcf.abc_svc.setFootSteps([OpenHRP.AutoBalancerService.Footstep([0,-0.09,0],[1,0,0,0],"rleg"),OpenHRP.AutoBalancerService.Footstep([0,0.0,0],[1,0,0,0],"lleg")])
        hcf.abc_svc.waitFootSteps();
        hcf.abc_svc.setFootSteps([OpenHRP.AutoBalancerService.Footstep([0,-0.09,0],[1,0,0,0],"rleg"),OpenHRP.AutoBalancerService.Footstep([0,0.09,0],[1,0,0,0],"lleg")])
        hcf.abc_svc.waitFootSteps();
        print >> sys.stderr, "  => Successfully mask works. Legs joints stops when collision."
        print >> sys.stderr, "  5.2 Collision mask between leg and arm : Check RLEG_HIP_R and RARM_WRIST*"
        print >> sys.stderr, "      Desired behavior : Leg joints moves and arm joints stops when collision."
        hcf.seq_svc.setJointAngles(col_safe_pose, 1.0);
        hcf.waitInterpolation();
        hcf.abc_svc.goVelocity(0,0,0);
        hcf.seq_svc.setJointAngles(col_fail_pose, 1.0);
        hcf.waitInterpolation();
        hcf.seq_svc.setJointAngles(col_safe_pose, 3.0);
        hcf.waitInterpolation();
        hcf.abc_svc.goStop();
        print >> sys.stderr, "  => Successfully mask works. Arm joints stops and leg joints moves."
        print >> sys.stderr, "  5.3 Collision mask between leg and arm : Check RLEG_HIP_R and RARM_WRIST* and RLEG_ANKLE_R and LLEG_ANKLE_R (combination of 5.1 and 5.2)"
        print >> sys.stderr, "      Desired behavior : First, arm stops and legs moves."
        hcf.seq_svc.setJointAngles(col_safe_pose, 1.0);
        hcf.waitInterpolation();
        print >> sys.stderr, "      Desired behavior : Next, arm keeps stopping and legs stops."
        hcf.abc_svc.setFootSteps([OpenHRP.AutoBalancerService.Footstep([0,-0.09,0],[1,0,0,0],"rleg"),OpenHRP.AutoBalancerService.Footstep([0,0.0,0],[1,0,0,0],"lleg")])
        hcf.abc_svc.waitFootSteps();
        hcf.abc_svc.setFootSteps([OpenHRP.AutoBalancerService.Footstep([0,-0.09,0],[1,0,0,0],"rleg"),OpenHRP.AutoBalancerService.Footstep([0,0.09,0],[1,0,0,0],"lleg")])
        hcf.abc_svc.waitFootSteps();
        print >> sys.stderr, "  => Successfully mask works with combined situation."
        hcf.stopAutoBalancer()

def demo():
    init()
    demoCollisionCheckSafe()
    demoCollisionCheckFail()
    demoCollisionCheckFailWithSetTolerance()
    demoCollisionDisableEnable()
    #demoCollisionMask()

if __name__ == '__main__':
    demo()

#!/usr/bin/env python

try:
    from hrpsys.hrpsys_config import *
    import OpenHRP
except:
    print("import without hrpsys")
    import rtm
    from rtm import *
    from OpenHRP import *
    import waitInput
    from waitInput import *
    import socket
    import time

def vector_equal_eps (vec1, vec2, eps=1e-5):
    if len(vec1) == len(vec2):
        for e1, e2 in zip(vec1, vec2):
            if abs(e1 - e2) > eps:
                return False
        return True
    else:
        return False

def init ():
    global hcf, init_pose, col_safe_pose, col_fail_pose, hrpsys_version
    hcf = HrpsysConfigurator()
    hcf.getRTCList = hcf.getRTCListUnstable
    hcf.init ("SampleRobot(Robot)0", "$(PROJECT_DIR)/../model/sample1.wrl")
    init_pose = [0]*29
    col_safe_pose = [0.0,-0.349066,0.0,0.820305,-0.471239,0.0,0.523599,0.0,0.0,-1.74533,0.15708,-0.113446,0.0,0.0,-0.349066,0.0,0.820305,-0.471239,0.0,0.523599,0.0,0.0,-1.74533,-0.15708,-0.113446,0.0,0.0,0.0,0.0]
    col_fail_pose = [0.0,-0.349066,0.0,0.820305,-0.471239,0.0,0.845363,0.03992,0.250074,-1.32816,0.167513,0.016204,0.0,0.0,-0.349066,0.0,0.820305,-0.471239,0.0,0.523599,0.0,0.0,-1.74533,-0.15708,-0.113446,0.0,0.0,0.0,0.0]
    hrpsys_version = hcf.co.ref.get_component_profile().version
    print("hrpsys_version = %s"%hrpsys_version, file=sys.stderr)

# demo functions
def demoCollisionCheckSafe ():
    print("1. CollisionCheck in safe pose", file=sys.stderr)
    hcf.seq_svc.setJointAngles(col_safe_pose, 3.0);
    hcf.waitInterpolation();
    counter = 0
    while (counter < 20) and (not vector_equal_eps([x / 180 * math.pi  for x in hcf.getJointAngles()], col_safe_pose)):
        time.sleep(0.2)
        counter = counter + 1
    assert(counter != 20)
    cs=hcf.co_svc.getCollisionStatus()[1]
    if cs.safe_posture:
        print("  => Safe pose", file=sys.stderr)
    assert(cs.safe_posture is True)

def demoCollisionCheckFail ():
    print("2. CollisionCheck in fail pose", file=sys.stderr)
    hcf.seq_svc.setJointAngles(col_fail_pose, 3.0);
    hcf.waitInterpolation();
    cs=hcf.co_svc.getCollisionStatus()[1]
    if not cs.safe_posture:
        print("  => Successfully stop fail pose", file=sys.stderr)
    assert((not cs.safe_posture) is True)
    hcf.seq_svc.setJointAngles(col_safe_pose, 3.0);
    hcf.waitInterpolation();
    cs=hcf.co_svc.getCollisionStatus()[1]
    if cs.safe_posture:
        print("  => Successfully return to safe pose", file=sys.stderr)
    assert(cs.safe_posture is True)

def demoCollisionCheckFailWithSetTolerance ():
    print("3. CollisionCheck in fail pose with 0.1[m] tolerance", file=sys.stderr)
    hcf.co_svc.setTolerance("all", 0.1); # [m]
    hcf.seq_svc.setJointAngles(col_fail_pose, 1.0);
    hcf.waitInterpolation();
    cs=hcf.co_svc.getCollisionStatus()[1]
    if not cs.safe_posture:
        print("  => Successfully stop fail pose (0.1[m] tolerance)", file=sys.stderr)
    assert((not cs.safe_posture) is True)
    hcf.co_svc.setTolerance("all", 0.0); # [m]
    hcf.seq_svc.setJointAngles(col_safe_pose, 3.0);
    hcf.waitInterpolation();
    cs=hcf.co_svc.getCollisionStatus()[1]
    if cs.safe_posture:
        print("  => Successfully return to safe pose", file=sys.stderr)
    assert(cs.safe_posture is True)

def demoCollisionDisableEnable ():
    print("4. CollisionDetection enable and disable", file=sys.stderr)
    hcf.seq_svc.setJointAngles(col_safe_pose, 1.0);
    hcf.waitInterpolation();
    if hcf.co_svc.disableCollisionDetection():
        print("  => Successfully disabled when no collision", file=sys.stderr)
    assert(hcf.co_svc.disableCollisionDetection() is True)
    if hcf.co_svc.enableCollisionDetection():
        print("  => Successfully enabled when no collision", file=sys.stderr)
    assert(hcf.co_svc.enableCollisionDetection() is True)
    hcf.seq_svc.setJointAngles(col_fail_pose, 1.0);
    hcf.waitInterpolation();
    if not hcf.co_svc.disableCollisionDetection():
        print("  => Successfully inhibit disabling when collision", file=sys.stderr)
    assert((not hcf.co_svc.disableCollisionDetection()) is True)
    hcf.seq_svc.setJointAngles(col_safe_pose, 1.0);
    hcf.waitInterpolation();

def demoCollisionMask ():
    if hcf.abc_svc != None:
        print("5. Collision mask test", file=sys.stderr)
        hcf.co_svc.setTolerance("all", 0); # [m]
        hcf.startAutoBalancer()
        print("  5.1 Collision mask among legs : Check RLEG_ANKLE_R - LLEG_ANKLE_R", file=sys.stderr)
        print("      Desired behavior : Robot stops when legs collision.", file=sys.stderr)
        hcf.setFootSteps([OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0,-0.09,0],[1,0,0,0],"rleg")]),
                          OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0,0.0,0],[1,0,0,0],"lleg")])])
        hcf.abc_svc.waitFootSteps();
        hcf.setFootSteps([OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0,-0.09,0],[1,0,0,0],"rleg")]),
                          OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0,0.09,0],[1,0,0,0],"lleg")])])
        hcf.abc_svc.waitFootSteps();
        print("  => Successfully mask works. Legs joints stops when collision.", file=sys.stderr)
        print("  5.2 Collision mask between leg and arm : Check RLEG_HIP_R and RARM_WRIST*", file=sys.stderr)
        print("      Desired behavior : Leg joints moves and arm joints stops when collision.", file=sys.stderr)
        hcf.seq_svc.setJointAngles(col_safe_pose, 1.0);
        hcf.waitInterpolation();
        hcf.abc_svc.goVelocity(0,0,0);
        hcf.seq_svc.setJointAngles(col_fail_pose, 1.0);
        hcf.waitInterpolation();
        hcf.seq_svc.setJointAngles(col_safe_pose, 3.0);
        hcf.waitInterpolation();
        hcf.abc_svc.goStop();
        print("  => Successfully mask works. Arm joints stops and leg joints moves.", file=sys.stderr)
        print("  5.3 Collision mask between leg and arm : Check RLEG_HIP_R and RARM_WRIST* and RLEG_ANKLE_R and LLEG_ANKLE_R (combination of 5.1 and 5.2)", file=sys.stderr)
        print("      Desired behavior : First, arm stops and legs moves.", file=sys.stderr)
        hcf.seq_svc.setJointAngles(col_safe_pose, 1.0);
        hcf.waitInterpolation();
        print("      Desired behavior : Next, arm keeps stopping and legs stops.", file=sys.stderr)
        hcf.setFootSteps([OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0,-0.09,0],[1,0,0,0],"rleg")]),
                          OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0,0.0,0],[1,0,0,0],"lleg")])])
        hcf.abc_svc.waitFootSteps();
        hcf.setFootSteps([OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0,-0.09,0],[1,0,0,0],"rleg")]),
                          OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0,0.09,0],[1,0,0,0],"lleg")])])
        hcf.abc_svc.waitFootSteps();
        print("  => Successfully mask works with combined situation.", file=sys.stderr)
        hcf.stopAutoBalancer()

def demo():
    init()
    demoCollisionCheckSafe()
    demoCollisionCheckFail()
    demoCollisionCheckFailWithSetTolerance()
    demoCollisionDisableEnable()
    #demoCollisionMask()

def demo_co_loop():
    init()
    from packaging.version import parse as StrictVersion
    if StrictVersion(hrpsys_version) >= StrictVersion('315.10.0'):
        demoCollisionCheckSafe()
        demoCollisionCheckFail()
        demoCollisionCheckFailWithSetTolerance()
        demoCollisionDisableEnable()

if __name__ == '__main__':
    demo()

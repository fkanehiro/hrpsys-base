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

def vector_equal_eps (vec1, vec2, eps=1e-5):
    if len(vec1) == len(vec2):
        for e1, e2 in zip(vec1, vec2):
            if abs(e1 - e2) > eps:
                return False
        return True
    else:
        return False

def init ():
    global hcf, init_pose, col_safe_pose, col_fail_pose, hrpsys_version, fout, curr_loop

    fout = open('result.txt', 'w')
    fout.write('Condition\tLoop\tComp Time\tRecov Time\tLoop for check\n')

    hcf = HrpsysConfigurator()
    hcf.getRTCList = hcf.getRTCListUnstable
    hcf.init ("HiroNX(Robot)0")

    curr_loop = 1
    hcf.co_svc.setCollisionLoop(curr_loop)

    init_pose = [0]*29
    # col_safe_pose = [0.0,-0.349066,0.0,0.820305,-0.471239,0.0,0.523599,0.0,0.0,-1.74533,0.15708,-0.113446,0.0,0.0,-0.349066,0.0,0.820305,-0.471239,0.0,0.523599,0.0,0.0,-1.74533,-0.15708,-0.113446,0.0,0.0,0.0,0.0]
    # col_fail_pose = [0.0,-0.349066,0.0,0.820305,-0.471239,0.0,0.845363,0.03992,0.250074,-1.32816,0.167513,0.016204,0.0,0.0,-0.349066,0.0,0.820305,-0.471239,0.0,0.523599,0.0,0.0,-1.74533,-0.15708,-0.113446,0.0,0.0,0.0,0.0]
    col_safe_pose = [0.0, 0.0, 0.0, 
        0.0, -0.010471975511965976, 0.0, 
        -1.7453292519943295, 0.26529004630313807, 0.16406094968746698, 
        0.05585053606381855, 0.0, 0.0, 
        0.0, 0.0, 0.010471975511963548, 
        0.0, -1.745329251994327, -0.265290046303138, 
        0.16406094968746654, -0.055850536063817735, 0.0, 
        0.0, 0.0, 0.0]
    col_fail_pose = [0.0, 0.0, 0.0, 
        0.0, -0.010471975511965976, 0.0, 
        -1.7453292519943295, 0.26529004630313807, 0.16406094968746698, 
        0.05585053606381855, 0.0, 0.0, 
        0.0, 0.0, -0.9357196648099176, 
        -0.5020351200853644, -0.7480480183116466, -0.15644273591164157, 
        -0.10807458370637157, 0.9688350378358652, 0.0, 
        0.0, 0.0, 0.0]
    hrpsys_version = hcf.co.ref.get_component_profile().version
    print >> sys.stderr, "hrpsys_version = %s" % hrpsys_version

def printCollisionState(cs):
    print >> sys.stderr, "Collision State:\n"
    print >> sys.stderr, "Time: %f" % cs.time
    print >> sys.stderr, "Computation time: %f" % cs.computation_time
    print >> sys.stderr, "Safe Posture: %s" % cs.safe_posture
    print >> sys.stderr, "Recover time: %f" % cs.recover_time
    print >> sys.stderr, "Loop for check: %d" % cs.loop_for_check
    # print >> sys.stderr, cs.angle
    # print >> sys.stderr, cs.collide
    # print >> sys.stderr, cs.lines

def outputCollisionState(cs, condition):
	s = "%.1f\t\t\t%d\t\t%f\t%f\t%d\n" % (condition, curr_loop, cs.computation_time, cs.recover_time, cs.loop_for_check)
	fout.write(s)

# demo functions
def demoCollisionCheckSafe ():
    print >> sys.stderr, "1. CollisionCheck in safe pose"
    hcf.seq_svc.setJointAngles(col_safe_pose, 3.0);
    hcf.waitInterpolation();
    counter = 0
    while (counter < 20) and (not vector_equal_eps([x / 180 * math.pi  for x in hcf.getJointAngles()], col_safe_pose)):
        time.sleep(0.2)
        counter = counter + 1
        # print >> sys.stderr, counter
    assert(counter != 20)
    cs = hcf.co_svc.getCollisionStatus()[1]
    if cs.safe_posture:
        outputCollisionState(cs, 1)
        print >> sys.stderr, "  => Safe pose"
    assert(cs.safe_posture is True)

def demoCollisionCheckFail ():
    print >> sys.stderr, "2. CollisionCheck in fail pose"
    hcf.seq_svc.setJointAngles(col_fail_pose, 3.0);
    hcf.waitInterpolation();
    cs = hcf.co_svc.getCollisionStatus()[1]
    if not cs.safe_posture:
        outputCollisionState(cs, 2.0)
        print >> sys.stderr, "  => Successfully stop fail pose"
    assert((not cs.safe_posture) is True)
    hcf.seq_svc.setJointAngles(col_safe_pose, 3.0);
    hcf.waitInterpolation();
    cs=hcf.co_svc.getCollisionStatus()[1]
    if cs.safe_posture:
        outputCollisionState(cs, 2.1)
        print >> sys.stderr, "  => Successfully return to safe pose"
    assert(cs.safe_posture is True)

def demoCollisionCheckFailWithSetTolerance ():
    print >> sys.stderr, "3. CollisionCheck in fail pose with 0.1[m] tolerance"
    hcf.co_svc.setTolerance("all", 0.1); # [m]
    hcf.seq_svc.setJointAngles(col_fail_pose, 1.0);
    hcf.waitInterpolation();
    cs = hcf.co_svc.getCollisionStatus()[1]
    if not cs.safe_posture:
        outputCollisionState(cs, 3.0)
        print >> sys.stderr, "  => Successfully stop fail pose (0.1[m] tolerance)"
    assert((not cs.safe_posture) is True)
    hcf.co_svc.setTolerance("all", 0.0); # [m]
    hcf.seq_svc.setJointAngles(col_safe_pose, 3.0);
    hcf.waitInterpolation();
    cs = hcf.co_svc.getCollisionStatus()[1]
    if cs.safe_posture:
        outputCollisionState(cs, 3.1)
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

def demo():
    init()
    demoCollisionCheckSafe()
    demoCollisionCheckFail()
    demoCollisionCheckFailWithSetTolerance()
    demoCollisionDisableEnable()
    #demoCollisionMask()

def demo_co_loop():
    init()
    from distutils.version import StrictVersion
    if StrictVersion(hrpsys_version) >= StrictVersion('315.10.0'):
        demoCollisionCheckSafe()
        demoCollisionCheckFail()
        demoCollisionCheckFailWithSetTolerance()
        demoCollisionDisableEnable()

def demo_loop_change():
    init()
    for i in range(2, 30):
    	global curr_loop
    	curr_loop = i
    	hcf.co_svc.setCollisionLoop(curr_loop)
        demoCollisionCheckSafe()
        demoCollisionCheckFail()
        demoCollisionCheckFailWithSetTolerance()
        demoCollisionDisableEnable()

    fout.close()

if __name__ == '__main__':
    # demo()
    # demo_co_loop()
    demo_loop_change()

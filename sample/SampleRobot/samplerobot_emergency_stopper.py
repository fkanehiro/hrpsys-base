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
    global hcf, init_pose, reset_pose
    hcf = HrpsysConfigurator()
    hcf.getRTCList = hcf.getRTCListUnstable
    hcf.init ("SampleRobot(Robot)0", "$(PROJECT_DIR)/../model/sample1.wrl")
    init_pose = [0]*29
    reset_pose = [0.0,-0.772215,0.0,1.8338,-1.06158,0.0,0.523599,0.0,0.0,-2.44346,0.15708,-0.113446,0.637045,0.0,-0.772215,0.0,1.8338,-1.06158,0.0,0.523599,0.0,0.0,-2.44346,-0.15708,-0.113446,-0.637045,0.0,0.0,0.0]

def angleDistance (angle1, angle2):
    return sum([abs(i-j) for (i,j) in zip(angle1,angle2)])

def angleApproxEqual (angle1, angle2, thre=1e-3):
    return angleDistance(angle1, angle2) < thre

# demo functions
def demoEmergencyStop ():
    print >> sys.stderr, "1. test stopMotion and releaseMotion"
    hcf.es_svc.releaseMotion()
    hcf.seq_svc.setJointAngles(init_pose, 1.0)
    hcf.waitInterpolation()
    time.sleep(0.1)
    tmp_angle1 = hcf.getActualState().angle
    play_time = 10
    hcf.seq_svc.setJointAngles(reset_pose, play_time)
    print >> sys.stderr, "  send angle_vector of %d [sec]" % play_time
    time.sleep(4)
    print >> sys.stderr, "  check whether robot pose is changing"
    tmp_angle2 = hcf.getActualState().angle
    if angleApproxEqual(init_pose, tmp_angle1) and not(angleApproxEqual(tmp_angle1, tmp_angle2)):
        print >> sys.stderr, "  => robot is moving."
    assert (angleApproxEqual(init_pose, tmp_angle1) and not(angleApproxEqual(tmp_angle1, tmp_angle2)))
    print >> sys.stderr, "  stop motion"
    hcf.es_svc.stopMotion()
    time.sleep(0.1)
    print >> sys.stderr, "  check whether robot pose remained still"
    tmp_angle1 = hcf.getActualState().angle
    time.sleep(3)
    tmp_angle2 = hcf.getActualState().angle
    if angleApproxEqual(tmp_angle1, tmp_angle2):
        print >> sys.stderr, "  => robot is not moving. stopMotion is working succesfully."
    assert (angleApproxEqual(tmp_angle1, tmp_angle2))
    print >> sys.stderr, "  release motion"
    hcf.es_svc.releaseMotion()
    print >> sys.stderr, "  check whether robot pose changed"
    tmp_angle1 = hcf.getActualState().angle
    hcf.waitInterpolation()
    time.sleep(0.1)
    tmp_angle2 = hcf.getActualState().angle
    if (not(angleApproxEqual(tmp_angle1, tmp_angle2)) and angleApproxEqual(tmp_angle2, reset_pose)):
        print >> sys.stderr, "  => robot is moving. releaseMotion is working succesfully."
    assert(not(angleApproxEqual(tmp_angle1, tmp_angle2)) and angleApproxEqual(tmp_angle2, reset_pose))

def demoEmergencyStopWithKeyInteracton ():
    print >> sys.stderr, "1. test stopMotion and releaseMotion with key interaction"
    pose_list = [reset_pose, init_pose] * 4
    play_time = 5
    hcf.seq_svc.playPattern(pose_list, [[0,0,0]]*len(pose_list), [[0,0,0]]*len(pose_list), [play_time]*len(pose_list))
    print >> sys.stderr, "  send angle_vector_sequence of %d [sec]" % (play_time*len(pose_list))
    print >> sys.stderr, "  press Enter to stop / release motion"
    while True:
        raw_input()
        print >> sys.stderr, "  stop motion"
        hcf.es_svc.stopMotion()
        if hcf.seq_svc.isEmpty(): break
        raw_input()
        print >> sys.stderr, "  release motion"
        hcf.es_svc.releaseMotion()
        if hcf.seq_svc.isEmpty(): break
    hcf.es_svc.releaseMotion()

def demoEmergencyStopReleaseWhenDeactivated():
    print >> sys.stderr, "2. test transition to release mode when deactivated"
    print >> sys.stderr, "  stop motion"
    hcf.es_svc.stopMotion()
    hcf.hes_svc.stopMotion()
    if((hcf.es_svc.getEmergencyStopperParam()[1].is_stop_mode == True) and (hcf.hes_svc.getEmergencyStopperParam()[1].is_stop_mode == True)):
        print >> sys.stderr, "  emergency stopper become stop mode succesfully"
    print >> sys.stderr, "  deactivate and activate es and hes"
    hcf.es.stop()
    hcf.hes.stop()
    hcf.es.start()
    hcf.hes.start()
    if((hcf.es_svc.getEmergencyStopperParam()[1].is_stop_mode == False) and (hcf.hes_svc.getEmergencyStopperParam()[1].is_stop_mode == False)):
        print >> sys.stderr, "  emergency stopper become release mode succesfully"
    assert((hcf.es_svc.getEmergencyStopperParam()[1].is_stop_mode == False) and (hcf.hes_svc.getEmergencyStopperParam()[1].is_stop_mode == False))

def demo(key_interaction=False):
    init()
    if key_interaction:
        demoEmergencyStopWithKeyInteracton()
    else:
        demoEmergencyStop()
    demoEmergencyStopReleaseWhenDeactivated()

if __name__ == '__main__':
    demo()

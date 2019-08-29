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
    global hcf, init_pose, reset_pose, wrench_command0, wrench_command1, hrpsys_version
    hcf = HrpsysConfigurator()
    hcf.getRTCList = hcf.getRTCListUnstable
    hcf.init ("SampleRobot(Robot)0", "$(PROJECT_DIR)/../model/sample1.wrl")
    if hcf.es != None:
        for sen in filter(lambda x: x.type == "Force", hcf.sensors):
            hcf.connectLoggerPort(hcf.es, sen.name+"Out")
    init_pose = [0]*29
    reset_pose = [0.0,-0.772215,0.0,1.8338,-1.06158,0.0,0.523599,0.0,0.0,-2.44346,0.15708,-0.113446,0.637045,0.0,-0.772215,0.0,1.8338,-1.06158,0.0,0.523599,0.0,0.0,-2.44346,-0.15708,-0.113446,-0.637045,0.0,0.0,0.0]
    wrench_command0 = [0.0]*24
    wrench_command1 = [1.0]*24
    hrpsys_version = hcf.seq.ref.get_component_profile().version
    print("hrpsys_version = %s"%hrpsys_version)

def arrayDistance (angle1, angle2):
    return sum([abs(i-j) for (i,j) in zip(angle1,angle2)])

def arrayApproxEqual (angle1, angle2, thre=1e-3):
    return arrayDistance(angle1, angle2) < thre

def saveLogForCheckParameter(log_fname="/tmp/test-samplerobot-emergency-stopper-check-param"):
    hcf.setMaxLogLength(1);hcf.clearLog();time.sleep(0.1);hcf.saveLog(log_fname)

def checkParameterFromLog(port_name, log_fname="/tmp/test-samplerobot-emergency-stopper-check-param", save_log=True, rtc_name="es"):
    if save_log:
        saveLogForCheckParameter(log_fname)
    return map(float, open(log_fname+"."+rtc_name+"_"+port_name, "r").readline().split(" ")[1:-1])

def getWrenchArray ():
    saveLogForCheckParameter()
    return reduce(lambda x,y: x+y, (map(lambda fs : checkParameterFromLog(fs+"Out", save_log=False), ['lfsensor', 'rfsensor', 'lhsensor', 'rhsensor'])))

# demo functions
def demoEmergencyStopJointAngle ():
    print >> sys.stderr, "1. test stopMotion and releaseMotion for joint angle"
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
    if arrayApproxEqual(init_pose, tmp_angle1) and not(arrayApproxEqual(tmp_angle1, tmp_angle2)):
        print >> sys.stderr, "  => robot is moving."
    assert (arrayApproxEqual(init_pose, tmp_angle1) and not(arrayApproxEqual(tmp_angle1, tmp_angle2)))
    print >> sys.stderr, "  stop motion"
    hcf.es_svc.stopMotion()
    time.sleep(0.1)
    print >> sys.stderr, "  check whether robot pose remained still"
    tmp_angle1 = hcf.getActualState().angle
    time.sleep(3)
    tmp_angle2 = hcf.getActualState().angle
    if arrayApproxEqual(tmp_angle1, tmp_angle2):
        print >> sys.stderr, "  => robot is not moving. stopMotion is working succesfully."
    assert (arrayApproxEqual(tmp_angle1, tmp_angle2))
    print >> sys.stderr, "  release motion"
    hcf.es_svc.releaseMotion()
    print >> sys.stderr, "  check whether robot pose changed"
    tmp_angle1 = hcf.getActualState().angle
    hcf.waitInterpolation()
    time.sleep(0.1)
    tmp_angle2 = hcf.getActualState().angle
    if (not(arrayApproxEqual(tmp_angle1, tmp_angle2)) and arrayApproxEqual(tmp_angle2, reset_pose)):
        print >> sys.stderr, "  => robot is moving. releaseMotion is working succesfully."
    assert(not(arrayApproxEqual(tmp_angle1, tmp_angle2)) and arrayApproxEqual(tmp_angle2, reset_pose))
    hcf.es_svc.releaseMotion()
    hcf.seq_svc.setJointAngles(init_pose, 1.0)
    hcf.waitInterpolation()

def demoEmergencyStopJointAngleWithKeyInteracton ():
    print >> sys.stderr, "1. test stopMotion and releaseMotion with key interaction for joint angle"
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

def demoEmergencyStopWrench ():
    print >> sys.stderr, "2. test stopMotion and releaseMotion for wrench"
    hcf.es_svc.releaseMotion()
    hcf.seq_svc.setJointAngles(init_pose, 1.0)
    hcf.seq_svc.setWrenches(wrench_command0, 1.0)
    hcf.waitInterpolation()
    time.sleep(0.1)
    tmp_wrench1 = getWrenchArray()
    play_time = 10
    hcf.seq_svc.setWrenches(wrench_command1, play_time)
    print >> sys.stderr, "  send wrench command of %d [sec]" % play_time
    time.sleep(4)
    print >> sys.stderr, "  check whether wrench is changing"
    tmp_wrench2 = getWrenchArray()
    if arrayApproxEqual(wrench_command0, tmp_wrench1) and not(arrayApproxEqual(tmp_wrench1, tmp_wrench2)):
        print >> sys.stderr, "  => wrench is changing."
    assert (arrayApproxEqual(wrench_command0, tmp_wrench1) and not(arrayApproxEqual(tmp_wrench1, tmp_wrench2)))
    print >> sys.stderr, "  stop motion"
    hcf.es_svc.stopMotion()
    time.sleep(0.1)
    print >> sys.stderr, "  check whether wrench remained still"
    tmp_wrench1 = getWrenchArray()
    time.sleep(3)
    tmp_wrench2 = getWrenchArray()
    if arrayApproxEqual(tmp_wrench1, tmp_wrench2):
        print >> sys.stderr, "  => wrench is not changing. stopMotion is working succesfully."
    assert (arrayApproxEqual(tmp_wrench1, tmp_wrench2))
    print >> sys.stderr, "  release motion"
    hcf.es_svc.releaseMotion()
    print >> sys.stderr, "  check whether wrench changed"
    tmp_wrench1 = getWrenchArray()
    hcf.waitInterpolation()
    time.sleep(1.0)
    tmp_wrench2 = getWrenchArray()
    if (not(arrayApproxEqual(tmp_wrench1, tmp_wrench2)) and arrayApproxEqual(tmp_wrench2, wrench_command1)):
        print >> sys.stderr, "  => wrench is changing. releaseMotion is working succesfully."
    assert(not(arrayApproxEqual(tmp_wrench1, tmp_wrench2)) and arrayApproxEqual(tmp_wrench2, wrench_command1))
    hcf.es_svc.releaseMotion()
    hcf.seq_svc.setWrenches(wrench_command0, 1.0)
    hcf.waitInterpolation()

def demoEmergencyStopReleaseWhenDeactivated():
    print >> sys.stderr, "3. test transition to release mode when deactivated"
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
    from distutils.version import StrictVersion
    if StrictVersion(hrpsys_version) >= StrictVersion('315.6.0'):
        if key_interaction:
            demoEmergencyStopJointAngleWithKeyInteracton()
        else:
            demoEmergencyStopJointAngle()
        demoEmergencyStopWrench()
        demoEmergencyStopReleaseWhenDeactivated()

if __name__ == '__main__':
    demo()

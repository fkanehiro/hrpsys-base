#!/usr/bin/env python
from functools import reduce

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

def init ():
    global hcf, init_pose, reset_pose, wrench_command0, wrench_command1, hrpsys_version
    hcf = HrpsysConfigurator()
    hcf.getRTCList = hcf.getRTCListUnstable
    hcf.init ("SampleRobot(Robot)0", "$(PROJECT_DIR)/../model/sample1.wrl")
    if hcf.es != None:
        for sen in [x for x in hcf.sensors if x.type == "Force"]:
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
    return list(map(float, open(log_fname+"."+rtc_name+"_"+port_name, "r").readline().split(" ")[1:-1]))

def getWrenchArray ():
    saveLogForCheckParameter()
    return reduce(lambda x,y: x+y, ([checkParameterFromLog(fs+"Out", save_log=False) for fs in ['lfsensor', 'rfsensor', 'lhsensor', 'rhsensor']]))

# demo functions
def demoEmergencyStopJointAngle ():
    print("1. test stopMotion and releaseMotion for joint angle", file=sys.stderr)
    hcf.es_svc.releaseMotion()
    hcf.seq_svc.setJointAngles(init_pose, 1.0)
    hcf.waitInterpolation()
    time.sleep(0.1)
    tmp_angle1 = hcf.getActualState().angle
    play_time = 10
    hcf.seq_svc.setJointAngles(reset_pose, play_time)
    print("  send angle_vector of %d [sec]" % play_time, file=sys.stderr)
    time.sleep(4)
    print("  check whether robot pose is changing", file=sys.stderr)
    tmp_angle2 = hcf.getActualState().angle
    if arrayApproxEqual(init_pose, tmp_angle1) and not(arrayApproxEqual(tmp_angle1, tmp_angle2)):
        print("  => robot is moving.", file=sys.stderr)
    assert (arrayApproxEqual(init_pose, tmp_angle1) and not(arrayApproxEqual(tmp_angle1, tmp_angle2)))
    print("  stop motion", file=sys.stderr)
    hcf.es_svc.stopMotion()
    time.sleep(0.1)
    print("  check whether robot pose remained still", file=sys.stderr)
    tmp_angle1 = hcf.getActualState().angle
    time.sleep(3)
    tmp_angle2 = hcf.getActualState().angle
    if arrayApproxEqual(tmp_angle1, tmp_angle2):
        print("  => robot is not moving. stopMotion is working succesfully.", file=sys.stderr)
    assert (arrayApproxEqual(tmp_angle1, tmp_angle2))
    print("  release motion", file=sys.stderr)
    hcf.es_svc.releaseMotion()
    print("  check whether robot pose changed", file=sys.stderr)
    tmp_angle1 = hcf.getActualState().angle
    hcf.waitInterpolation()
    time.sleep(0.1)
    tmp_angle2 = hcf.getActualState().angle
    if (not(arrayApproxEqual(tmp_angle1, tmp_angle2)) and arrayApproxEqual(tmp_angle2, reset_pose)):
        print("  => robot is moving. releaseMotion is working succesfully.", file=sys.stderr)
    assert(not(arrayApproxEqual(tmp_angle1, tmp_angle2)) and arrayApproxEqual(tmp_angle2, reset_pose))
    hcf.es_svc.releaseMotion()
    hcf.seq_svc.setJointAngles(init_pose, 1.0)
    hcf.waitInterpolation()

def demoEmergencyStopJointAngleWithKeyInteracton ():
    print("1. test stopMotion and releaseMotion with key interaction for joint angle", file=sys.stderr)
    pose_list = [reset_pose, init_pose] * 4
    play_time = 5
    hcf.seq_svc.playPattern(pose_list, [[0,0,0]]*len(pose_list), [[0,0,0]]*len(pose_list), [play_time]*len(pose_list))
    print("  send angle_vector_sequence of %d [sec]" % (play_time*len(pose_list)), file=sys.stderr)
    print("  press Enter to stop / release motion", file=sys.stderr)
    while True:
        input()
        print("  stop motion", file=sys.stderr)
        hcf.es_svc.stopMotion()
        if hcf.seq_svc.isEmpty(): break
        input()
        print("  release motion", file=sys.stderr)
        hcf.es_svc.releaseMotion()
        if hcf.seq_svc.isEmpty(): break
    hcf.es_svc.releaseMotion()

def demoEmergencyStopWrench ():
    print("2. test stopMotion and releaseMotion for wrench", file=sys.stderr)
    hcf.es_svc.releaseMotion()
    hcf.seq_svc.setJointAngles(init_pose, 1.0)
    hcf.seq_svc.setWrenches(wrench_command0, 1.0)
    hcf.waitInterpolation()
    time.sleep(0.1)
    tmp_wrench1 = getWrenchArray()
    play_time = 10
    hcf.seq_svc.setWrenches(wrench_command1, play_time)
    print("  send wrench command of %d [sec]" % play_time, file=sys.stderr)
    time.sleep(4)
    print("  check whether wrench is changing", file=sys.stderr)
    tmp_wrench2 = getWrenchArray()
    if arrayApproxEqual(wrench_command0, tmp_wrench1) and not(arrayApproxEqual(tmp_wrench1, tmp_wrench2)):
        print("  => wrench is changing.", file=sys.stderr)
    assert (arrayApproxEqual(wrench_command0, tmp_wrench1) and not(arrayApproxEqual(tmp_wrench1, tmp_wrench2)))
    print("  stop motion", file=sys.stderr)
    hcf.es_svc.stopMotion()
    time.sleep(0.1)
    print("  check whether wrench remained still", file=sys.stderr)
    tmp_wrench1 = getWrenchArray()
    time.sleep(3)
    tmp_wrench2 = getWrenchArray()
    if arrayApproxEqual(tmp_wrench1, tmp_wrench2):
        print("  => wrench is not changing. stopMotion is working succesfully.", file=sys.stderr)
    assert (arrayApproxEqual(tmp_wrench1, tmp_wrench2))
    print("  release motion", file=sys.stderr)
    hcf.es_svc.releaseMotion()
    print("  check whether wrench changed", file=sys.stderr)
    tmp_wrench1 = getWrenchArray()
    hcf.waitInterpolation()
    time.sleep(1.0)
    tmp_wrench2 = getWrenchArray()
    if (not(arrayApproxEqual(tmp_wrench1, tmp_wrench2)) and arrayApproxEqual(tmp_wrench2, wrench_command1)):
        print("  => wrench is changing. releaseMotion is working succesfully.", file=sys.stderr)
    assert(not(arrayApproxEqual(tmp_wrench1, tmp_wrench2)) and arrayApproxEqual(tmp_wrench2, wrench_command1))
    hcf.es_svc.releaseMotion()
    hcf.seq_svc.setWrenches(wrench_command0, 1.0)
    hcf.waitInterpolation()

def demoEmergencyStopReleaseWhenDeactivated():
    print("3. test transition to release mode when deactivated", file=sys.stderr)
    print("  stop motion", file=sys.stderr)
    hcf.es_svc.stopMotion()
    hcf.hes_svc.stopMotion()
    if((hcf.es_svc.getEmergencyStopperParam()[1].is_stop_mode == True) and (hcf.hes_svc.getEmergencyStopperParam()[1].is_stop_mode == True)):
        print("  emergency stopper become stop mode succesfully", file=sys.stderr)
    print("  deactivate and activate es and hes", file=sys.stderr)
    hcf.es.stop()
    hcf.hes.stop()
    hcf.es.start()
    hcf.hes.start()
    if((hcf.es_svc.getEmergencyStopperParam()[1].is_stop_mode == False) and (hcf.hes_svc.getEmergencyStopperParam()[1].is_stop_mode == False)):
        print("  emergency stopper become release mode succesfully", file=sys.stderr)
    assert((hcf.es_svc.getEmergencyStopperParam()[1].is_stop_mode == False) and (hcf.hes_svc.getEmergencyStopperParam()[1].is_stop_mode == False))

def demo(key_interaction=False):
    init()
    from packaging.version import parse as StrictVersion
    if StrictVersion(hrpsys_version) >= StrictVersion('315.6.0'):
        if key_interaction:
            demoEmergencyStopJointAngleWithKeyInteracton()
        else:
            demoEmergencyStopJointAngle()
        demoEmergencyStopWrench()
        demoEmergencyStopReleaseWhenDeactivated()

if __name__ == '__main__':
    demo()

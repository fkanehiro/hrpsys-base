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

# demo functions
def demoEmergencyStop ():
    print "1. CollisionCheck in safe pose"

    print "send angle_vector_sequence"
    pose_list = [init_pose, reset_pose] * 5
    play_time = 4
    hcf.seq_svc.playPattern(pose_list, [[0,0,0]]*len(pose_list), [[0,0,0]]*len(pose_list), [play_time]*len(pose_list))

    print "press Enter to stop / release motion"
    while True:
        raw_input()
        print "stop motion"
        hcf.es_svc.stopMotion()
        if hcf.seq_svc.isEmpty(): break
        raw_input()
        print "release motion"
        hcf.es_svc.releaseMotion()
        if hcf.seq_svc.isEmpty(): break
    hcf.es_svc.releaseMotion()

def demo():
    init()
    demoEmergencyStop()

if __name__ == '__main__':
    demo()

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
    hrpsys_version = hcf.seq.ref.get_component_profile().version
    print("hrpsys_version = %s"%hrpsys_version)

def demo():
    init()

if __name__ == '__main__':
    demo()

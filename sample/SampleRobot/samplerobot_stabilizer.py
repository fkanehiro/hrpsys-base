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

def demo():
    init()
    # set initial pose from sample/controller/SampleController/etc/Sample.pos
    initial_pose = [-7.779e-005,  -0.378613,  -0.000209793,  0.832038,  -0.452564,  0.000244781,  0.31129,  -0.159481,  -0.115399,  -0.636277,  0,  0,  0,  -7.77902e-005,  -0.378613,  -0.000209794,  0.832038,  -0.452564,  0.000244781,  0.31129,  0.159481,  0.115399,  -0.636277,  0,  0,  0,  0,  0,  0]
    hcf.seq_svc.setJointAngles(initial_pose, 2.0)
    hcf.seq_svc.waitInterpolation()

    # 1. getParameter
    stp1 = hcf.st_svc.getParameter()
    print "getParameter() => OK"
    # 2. setParameter
    stp1.k_tpcc_p=[0.2, 0.2]
    stp1.k_tpcc_x=[4.0, 4.0]
    stp1.k_brot_p=[0.0, 0.0]
    hcf.st_svc.setParameter(stp1)
    stp2 = hcf.st_svc.getParameter()
    if stp1.k_tpcc_p == stp2.k_tpcc_p and stp1.k_tpcc_x == stp2.k_tpcc_x and stp1.k_brot_p == stp2.k_brot_p:
        print "setParameter() => OK"
    # 3. start and stop st
    hcf.st_svc.startStabilizer ()
    hcf.abc_svc.goPos(0.5, 0.1, 10)
    hcf.abc_svc.waitFootSteps()
    hcf.st_svc.stopStabilizer ()
    print "Start and Stop Stabilizer => OK"

if __name__ == '__main__':
    demo()

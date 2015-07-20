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
    global hcf, initial_pose
    # set initial pose from sample/controller/SampleController/etc/Sample.pos
    initial_pose = [-7.779e-005,  -0.378613,  -0.000209793,  0.832038,  -0.452564,  0.000244781,  0.31129,  -0.159481,  -0.115399,  -0.636277,  0,  0,  0,  -7.77902e-005,  -0.378613,  -0.000209794,  0.832038,  -0.452564,  0.000244781,  0.31129,  0.159481,  0.115399,  -0.636277,  0,  0,  0,  0,  0,  0]
    hcf = HrpsysConfigurator()
    hcf.getRTCList = hcf.getRTCListUnstable
    hcf.log_use_own_ec = True ### use own ec for simulation
    hcf.init ("SampleRobot(Robot)0", "$(PROJECT_DIR)/../model/sample1.wrl")

def demoSetMaxLogLength():
    print "1. Set max ring-buffer length : 200 [loop] * 0.002 [s] = 0.4 [s] data"
    hcf.log_svc.maxLength(200)
    print "  maxLength() =>OK"

def demoClearLog():
    print "2. Clear buffer"
    hcf.log_svc.clear()
    print "  clear() =>OK"

def demoSaveLog():
    print "3. Save log"
    hcf.seq_svc.setJointAngles(initial_pose, 2.0)
    hcf.seq_svc.waitInterpolation()
    # Save log files for each ports as /tmp/test-samplerobot-log.*
    #   file names are /tmp/test-samplerobot-log.[RTCName]_[PortName], c.f.,  /tmp/test-samplerobot-log.sh_qOut ... etc
    hcf.log_svc.save("/tmp/test-samplerobot-log")
    print "  save() =>OK"


def demo ():
    init()
    demoSetMaxLogLength()
    demoClearLog()
    demoSaveLog()

if __name__ == '__main__':
    demo()

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

def init ():
    global hcf, initial_pose
    hcf = HrpsysConfigurator()
    hcf.getRTCList = hcf.getRTCListUnstable
    hcf.log_use_own_ec = True ### use own ec for simulation
    hcf.init ("SampleRobot(Robot)0", "$(PROJECT_DIR)/../model/sample1.wrl")
    # set initial pose from sample/controller/SampleController/etc/Sample.pos
    initial_pose = [-7.779e-005,  -0.378613,  -0.000209793,  0.832038,  -0.452564,  0.000244781,  0.31129,  -0.159481,  -0.115399,  -0.636277,  0,  0,  0,  -7.77902e-005,  -0.378613,  -0.000209794,  0.832038,  -0.452564,  0.000244781,  0.31129,  0.159481,  0.115399,  -0.636277,  0,  0,  0,  0,  0,  0]
    hcf.seq_svc.setJointAngles(initial_pose, 2.0)
    hcf.waitInterpolation()

def demoSaveLog():
    print("1. Save log", file=sys.stderr)
    # Save log files for each ports as /tmp/test-samplerobot-log.*
    #   file names are /tmp/test-samplerobot-log.[RTCName]_[PortName], c.f.,  /tmp/test-samplerobot-log.sh_qOut ... etc
    hcf.saveLog("/tmp/test-samplerobot-log")
    ret = os.path.exists("/tmp/test-samplerobot-log.sh_qOut")
    if ret:
        print("  save() =>OK", file=sys.stderr)
    assert(ret is True)

def demoClearLog():
    print("2. Clear buffer", file=sys.stderr)
    hcf.clearLog()
    print("  clear() =>OK", file=sys.stderr)
    assert(True)

def demoSetMaxLogLength():
    print("3. Set max ring-buffer length : 100 [loop] * 0.002 [s] = 0.2 [s] data", file=sys.stderr)
    hcf.setMaxLogLength(100)
    hcf.seq_svc.setJointAngles(initial_pose, 0.2) # wait
    hcf.waitInterpolation()
    hcf.saveLog("/tmp/test-samplerobot-log")
    from subprocess import check_output
    ret = check_output(['wc', '-l', '/tmp/test-samplerobot-log.sh_qOut']).decode().split(" ")[0] == '100'
    if ret:
        print("  maxLength() =>OK", file=sys.stderr)
    assert(ret is True)

def demo ():
    init()
    demoSaveLog()
    demoClearLog()
    demoSetMaxLogLength()

if __name__ == '__main__':
    demo()

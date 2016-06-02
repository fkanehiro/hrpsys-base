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
    hrpsys_version = hcf.seq.ref.get_component_profile().version
    print("hrpsys_version = %s"%hrpsys_version)

# demo functions
def demoReferenceForceUpdater ():
    print >> sys.stderr, "1. get param"
    p = hcf.rfu_svc.getReferenceForceUpdaterParam()[1]
    print >> sys.stderr, "2. start/stop param for left arm"
    p.arm = 'larm'
    hcf.rfu_svc.setReferenceForceUpdaterParam(p)
    hcf.rfu_svc.startReferenceForceUpdater()
    time.sleep(1)
    hcf.rfu_svc.stopReferenceForceUpdater()

def demo():
    init()
    if hrpsys_version >= '315.9.0':
        demoReferenceForceUpdater()

if __name__ == '__main__':
    demo()

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

    # 1. get
    ret1=hcf.ic_svc.getImpedanceControllerParam("rarm")
    if ret1[0]:
        print "getImpedanceControllerParam => OK"
    # 2. set and start
    ret1[1].K_r=1.0
    ret1[1].D_r=2.0
    ret1[1].M_r=0.2
    ret2=hcf.ic_svc.setImpedanceControllerParam("rarm", ret1[1])
    ret3=hcf.ic_svc.getImpedanceControllerParam("rarm")
    if ret2:
        print "setImpedanceControllerParam => OK"
    # 3. start impedance
    ret4=hcf.ic_svc.startImpedanceController("rarm")
    if ret4:
        print "startImpedanceController => OK"
    # 4. set ref force and moment
    hcf.seq_svc.setWrenches([0,0,0,0,0,0,
                             0,0,0,0,0,0,
                             0,0,0,0,0,0,
                             20,10,-10,0,0,0,], 2.0);
    hcf.seq_svc.waitInterpolation();
    time.sleep(2)
    hcf.seq_svc.setWrenches([0,0,0,0,0,0,
                             0,0,0,0,0,0,
                             0,0,0,0,0,0,
                             0,0,0,0,0,0,], 2.0);
    hcf.seq_svc.waitInterpolation();
    time.sleep(2)
    hcf.seq_svc.setWrenches([0,0,0,0,0,0,
                             0,0,0,0,0,0,
                             0,0,0,0,0,0,
                             0,0,0,0.1,-0.1,0.1], 2.0);
    hcf.seq_svc.waitInterpolation();
    time.sleep(2)
    hcf.seq_svc.setWrenches([0,0,0,0,0,0,
                             0,0,0,0,0,0,
                             0,0,0,0,0,0,
                             0,0,0,0,0,0], 2.0);
    hcf.seq_svc.waitInterpolation();
    time.sleep(2)
    # 5. stop impedance
    ret5=hcf.ic_svc.stopImpedanceController("rarm")
    if ret5:
        print "stopImpedanceController => OK"

if __name__ == '__main__':
    demo()

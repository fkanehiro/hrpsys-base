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
    global hcf
    hcf = HrpsysConfigurator()
    hcf.getRTCList = hcf.getRTCListUnstable
    hcf.init ("SampleRobot(Robot)0", "$(PROJECT_DIR)/../model/sample1.wrl")

def demo ():
    init()
    # set initial pose from sample/controller/SampleController/etc/Sample.pos
    initial_pose = [-7.779e-005,  -0.378613,  -0.000209793,  0.832038,  -0.452564,  0.000244781,  0.31129,  -0.159481,  -0.115399,  -0.636277,  0,  0,  0,  -7.77902e-005,  -0.378613,  -0.000209794,  0.832038,  -0.452564,  0.000244781,  0.31129,  0.159481,  0.115399,  -0.636277,  0,  0,  0,  0,  0,  0]

    # 1. Check wrench data ports existence (real force sensor + virtual force sensor)
    hcf.seq_svc.setJointAngles(initial_pose, 2.0)
    hcf.seq_svc.waitInterpolation()
    fsensor_names = hcf.getForceSensorNames()
    # vs check
    port_names = fsensor_names
    if all([hcf.vs.port(x) for x in fsensor_names]):
        print(hcf.vs.name(), "ports are OK (", port_names, ")")
    # seq check
    port_names = [x+"Ref" for x in fsensor_names]
    if all([hcf.seq.port(x) for x in port_names]):
        print(hcf.seq.name(), "ports are OK (", port_names, ")")
    # sh check
    port_names = [x+"Out" for x in fsensor_names]
    if all([hcf.sh.port(x) for x in port_names]):
        print(hcf.sh.name(), "ports are OK (", port_names, ")")
    port_names = [x+"In" for x in fsensor_names]
    if all([hcf.sh.port(x) for x in port_names]):
        print(hcf.sh.name(), "ports are OK (", port_names, ")")
    # ic check
    port_names = ["ref_"+x+"In" for x in fsensor_names]
    if all([hcf.ic.port(x) for x in port_names]):
        print(hcf.ic.name(), "ports are OK (", port_names, ")")
    # abc check
    port_names = ["ref_"+x+"In" for x in fsensor_names]
    if all([hcf.ic.port(x) for x in port_names]):
        print(hcf.ic.name(), "ports are OK (", port_names, ")")

    # 2. Test impedance controller
    ret1=hcf.ic_svc.getImpedanceControllerParam("vrhsensor")
    ret1[1].base_name="CHEST"
    ret1[1].target_name="RARM_WRIST_P"
    ret1[1].K_r=1.0
    ret1[1].D_r=2.0
    ret1[1].M_r=0.2
    ret2=hcf.ic_svc.setImpedanceControllerParam("vrhsensor", ret1[1])
    hcf.seq_svc.setWrenches([0,0,0,0,0,0,
                             0,0,0,0,0,0,
                             0,0,0,0,0,0,
                             0,0,0,0,0,0,
                             0,0,0,0,0,0,
                             10,10,-10,0,0,0,], 2.0);
    hcf.seq_svc.waitInterpolation();
    hcf.seq_svc.setWrenches([0,0,0,0,0,0,
                             0,0,0,0,0,0,
                             0,0,0,0,0,0,
                             0,0,0,0,0,0,
                             0,0,0,0,0,0,
                             0,0,0,0,0,0,], 2.0);
    hcf.seq_svc.waitInterpolation();
    hcf.ic_svc.deleteImpedanceController("vrhsensor")
    print("test ImpedanceController for virtual force sensor => OK")

if __name__ == '__main__':
    demo()

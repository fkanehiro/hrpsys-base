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
    hcf = HrpsysConfigurator()
    hcf.getRTCList = hcf.getRTCListUnstable
    hcf.init ("SampleRobot(Robot)0", "$(PROJECT_DIR)/../model/sample1.wrl")
    # set initial pose from sample/controller/SampleController/etc/Sample.pos
    initial_pose = [-7.779e-005,  -0.378613,  -0.000209793,  0.832038,  -0.452564,  0.000244781,  0.31129,  -0.159481,  -0.115399,  -0.636277,  0,  0,  0,  -7.77902e-005,  -0.378613,  -0.000209794,  0.832038,  -0.452564,  0.000244781,  0.31129,  0.159481,  0.115399,  -0.636277,  0,  0,  0,  0,  0,  0]
    hcf.seq_svc.setJointAngles(initial_pose, 2.0)
    hcf.seq_svc.waitInterpolation()
    # Start AutoBalancer
    hcf.startAutoBalancer()
    # Remove offset
    for sen in ["rfsensor", "lfsensor"]:
        ofp=hcf.rmfo_svc.getForceMomentOffsetParam(sen)[1];
        ofp.link_offset_mass=1.9;ofp.link_offset_centroid=[0.08, 0, -0.03];
        hcf.rmfo_svc.setForceMomentOffsetParam(sen, ofp);

def calcCOP ():
    cop_info=rtm.readDataPort(hcf.st.port("COPInfo")).data
    lcopx=cop_info[1]/cop_info[2];lcopy=cop_info[0]/cop_info[2]
    rcopx=cop_info[1+3]/cop_info[2+3];rcopy=cop_info[0+3]/cop_info[2+3]
    return [[lcopx, lcopx], [rcopx, rcopy], # l cop, r cop
            [(cop_info[1]+cop_info[1+3])/(cop_info[2]+cop_info[2+3]),(cop_info[0]+cop_info[0+3])/(cop_info[2]+cop_info[2+3])]] # total ZMP

def demoGetParameter():
    print "1. getParameter"
    stp = hcf.st_svc.getParameter()
    print "  getParameter() => OK"

def demoSetParameter():
    print "2. setParameter"
    stp_org = hcf.st_svc.getParameter()
    stp = hcf.st_svc.getParameter()
    stp.k_tpcc_p=[0.2, 0.2]
    stp.k_tpcc_x=[4.0, 4.0]
    stp.k_brot_p=[0.0, 0.0]
    hcf.st_svc.setParameter(stp)
    stp = hcf.st_svc.getParameter()
    if stp.k_tpcc_p == stp_org.k_tpcc_p and stp.k_tpcc_x == stp_org.k_tpcc_x and stp.k_brot_p == stp_org.k_brot_p:
        print "  setParameter() => OK"

def demoStartStopST ():
    print "3. start and stop st"
    hcf.st_svc.startStabilizer ()
    hcf.abc_svc.goPos(0.5, 0.1, 10)
    hcf.abc_svc.waitFootSteps()
    hcf.st_svc.stopStabilizer ()
    print "  Start and Stop Stabilizer => OK"

def demoCOPCheck():
    stp=hcf.st_svc.getParameter()
    stp.eefm_leg_inside_margin=71.12*1e-3
    stp.eefm_leg_front_margin=182.0*1e-3
    stp.eefm_leg_rear_margin=72.0*1e-3
    hcf.st_svc.setParameter(stp)


def demo():
    init()

    demoGetParameter()
    demoSetParameter()
    #demoStartStopST()

if __name__ == '__main__':
    demo()

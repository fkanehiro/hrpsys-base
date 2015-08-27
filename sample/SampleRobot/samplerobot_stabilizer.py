#!/usr/bin/env python

from hrpsys.hrpsys_config import *
from hrpsys import OpenHRP
import sys
import math

def init ():
    global hcf, initial_pose, hrpsys_version
    hcf = HrpsysConfigurator()
    hcf.getRTCList = hcf.getRTCListUnstable
    hcf.init ("SampleRobot(Robot)0", "$(PROJECT_DIR)/../model/sample1.wrl")
    # set initial pose from sample/controller/SampleController/etc/Sample.pos
    initial_pose = [-7.779e-005,  -0.378613,  -0.000209793,  0.832038,  -0.452564,  0.000244781,  0.31129,  -0.159481,  -0.115399,  -0.636277,  0,  0,  0,  -7.77902e-005,  -0.378613,  -0.000209794,  0.832038,  -0.452564,  0.000244781,  0.31129,  0.159481,  0.115399,  -0.636277,  0,  0,  0,  0,  0,  0]
    hcf.seq_svc.setJointAngles(initial_pose, 2.0)
    hcf.seq_svc.waitInterpolation()
    hrpsys_version = hcf.seq.ref.get_component_profile().version
    print("hrpsys_version = %s"%hrpsys_version)
    if hrpsys_version >= '315.5.0':
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
    print >> sys.stderr, "1. getParameter"
    stp = hcf.st_svc.getParameter()
    print >> sys.stderr, "  getParameter() => OK"

def demoSetParameter():
    print >> sys.stderr, "2. setParameter"
    stp_org = hcf.st_svc.getParameter()
    # for tpcc
    stp_org.k_tpcc_p=[0.2, 0.2]
    stp_org.k_tpcc_x=[4.0, 4.0]
    stp_org.k_brot_p=[0.0, 0.0]
    # for eefm
    stp_org.eefm_leg_inside_margin=71.12*1e-3
    stp_org.eefm_leg_outside_margin=71.12*1e-3
    stp_org.eefm_leg_front_margin=182.0*1e-3
    stp_org.eefm_leg_rear_margin=72.0*1e-3
    stp_org.eefm_k1=[-1.39899,-1.39899]
    stp_org.eefm_k2=[-0.386111,-0.386111]
    stp_org.eefm_k3=[-0.175068,-0.175068]
    stp_org.eefm_rot_damping_gain=20*1.6*10 # Stiff parameter for simulation
    stp_org.eefm_pos_damping_gain=[3500*50, 3500*50, 3500*1.0*5] # Stiff parameter for simulation
    hcf.st_svc.setParameter(stp_org)
    stp = hcf.st_svc.getParameter()
    vcheck = stp.k_tpcc_p == stp_org.k_tpcc_p and stp.k_tpcc_x == stp_org.k_tpcc_x and stp.k_brot_p == stp_org.k_brot_p
    if vcheck:
        print >> sys.stderr, "  setParameter() => OK", vcheck
    assert(vcheck)

def checkActualBaseAttitude():
    rpy = rtm.readDataPort(hcf.rh.port("WAIST")).data.orientation
    ret = math.degrees(rpy.r) < 0.1 and math.degrees(rpy.p) < 0.1
    print >> sys.stderr, "  actual base rpy = ", ret, "(", rpy, ")"
    return ret

def demoStartStopTPCCST ():
    print >> sys.stderr, "3. start and stop TPCC st"
    if hcf.pdc:
        stp = hcf.st_svc.getParameter()
        stp.st_algorithm=OpenHRP.StabilizerService.TPCC
        hcf.st_svc.setParameter(stp)
        hcf.startStabilizer ()
        hcf.abc_svc.goPos(0.5, 0.1, 10)
        hcf.abc_svc.waitFootSteps()
        hcf.stopStabilizer ()
        ret = checkActualBaseAttitude()
        if ret:
            print >> sys.stderr, "  Start and Stop Stabilizer => OK"
        assert(ret)
    else:
        print >> sys.stderr, "  This sample is neglected in High-gain mode simulation"


def demoStartStopEEFMQPST ():
    print >> sys.stderr, "4. start and stop EEFMQP st"
    if hcf.pdc:
        stp = hcf.st_svc.getParameter()
        stp.st_algorithm=OpenHRP.StabilizerService.EEFMQP
        hcf.st_svc.setParameter(stp)
        hcf.startStabilizer ()
        hcf.abc_svc.goPos(0.3, 0, 0)
        hcf.abc_svc.waitFootSteps()
        hcf.stopStabilizer ()
        ret = checkActualBaseAttitude()
        if ret:
            print >> sys.stderr, "  Start and Stop Stabilizer => OK"
        assert(ret)
    else:
        print >> sys.stderr, "  This sample is neglected in High-gain mode simulation"

def demo():
    init()
    if hrpsys_version >= '315.5.0':
        demoGetParameter()
        demoSetParameter()
        demoStartStopTPCCST()
        demoStartStopEEFMQPST()

if __name__ == '__main__':
    demo()

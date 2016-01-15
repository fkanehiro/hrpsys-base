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
from subprocess import check_output

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
    tmp_leg_inside_margin=71.12*1e-3
    tmp_leg_outside_margin=71.12*1e-3
    tmp_leg_front_margin=182.0*1e-3
    tmp_leg_rear_margin=72.0*1e-3
    rleg_vertices = [OpenHRP.StabilizerService.TwoDimensionVertex(pos=[tmp_leg_front_margin, tmp_leg_inside_margin]),
                     OpenHRP.StabilizerService.TwoDimensionVertex(pos=[tmp_leg_front_margin, -1*tmp_leg_outside_margin]),
                     OpenHRP.StabilizerService.TwoDimensionVertex(pos=[-1*tmp_leg_rear_margin, -1*tmp_leg_outside_margin]),
                     OpenHRP.StabilizerService.TwoDimensionVertex(pos=[-1*tmp_leg_rear_margin, tmp_leg_inside_margin])]
    lleg_vertices = [OpenHRP.StabilizerService.TwoDimensionVertex(pos=[tmp_leg_front_margin, tmp_leg_outside_margin]),
                     OpenHRP.StabilizerService.TwoDimensionVertex(pos=[tmp_leg_front_margin, -1*tmp_leg_inside_margin]),
                     OpenHRP.StabilizerService.TwoDimensionVertex(pos=[-1*tmp_leg_rear_margin, -1*tmp_leg_inside_margin]),
                     OpenHRP.StabilizerService.TwoDimensionVertex(pos=[-1*tmp_leg_rear_margin, tmp_leg_outside_margin])]
    rarm_vertices = rleg_vertices
    larm_vertices = lleg_vertices
    stp_org.eefm_support_polygon_vertices_sequence = map (lambda x : OpenHRP.StabilizerService.SupportPolygonVertices(vertices=x), [lleg_vertices, rleg_vertices, larm_vertices, rarm_vertices])
    stp_org.eefm_leg_inside_margin=tmp_leg_inside_margin
    stp_org.eefm_leg_outside_margin=tmp_leg_outside_margin
    stp_org.eefm_leg_front_margin=tmp_leg_front_margin
    stp_org.eefm_leg_rear_margin=tmp_leg_rear_margin
    stp_org.eefm_k1=[-1.39899,-1.39899]
    stp_org.eefm_k2=[-0.386111,-0.386111]
    stp_org.eefm_k3=[-0.175068,-0.175068]
    stp_org.eefm_rot_damping_gain = [[20*1.6*1.5, 20*1.6*1.5, 1e5]]*4
    stp_org.eefm_pos_damping_gain = [[3500*50, 3500*50, 3500*1.0*1.5]]*4
    hcf.st_svc.setParameter(stp_org)
    stp = hcf.st_svc.getParameter()
    vcheck = stp.k_tpcc_p == stp_org.k_tpcc_p and stp.k_tpcc_x == stp_org.k_tpcc_x and stp.k_brot_p == stp_org.k_brot_p
    if vcheck:
        print >> sys.stderr, "  setParameter() => OK", vcheck
    assert(vcheck)

def saveLogForCheckParameter(log_fname="/tmp/test-samplerobot-stabilizer-check-param"):
    hcf.setMaxLogLength(1);hcf.clearLog();time.sleep(0.1);hcf.saveLog(log_fname)

def checkParameterFromLog(port_name, log_fname="/tmp/test-samplerobot-stabilizer-check-param", save_log=True, rtc_name="SampleRobot(Robot)0"):
    if save_log:
        saveLogForCheckParameter(log_fname)
    return map(float, open(log_fname+"."+rtc_name+"_"+port_name, "r").readline().split(" ")[1:-1])

def checkActualBaseAttitude(thre=5.0): # degree
    '''Check whether the robot falls down based on actual robot base-link attitude.
    '''
    act_rpy = checkParameterFromLog("WAIST")[3:]
    ret = abs(math.degrees(act_rpy[0])) < thre and abs(math.degrees(act_rpy[1])) < thre
    print >> sys.stderr, "  ret = ", ret, ", actual base rpy = (", act_rpy, ")"
    return ret

def demoStartStopTPCCST ():
    print >> sys.stderr, "3. start and stop TPCC st"
    if hcf.pdc:
        stp = hcf.st_svc.getParameter()
        stp.st_algorithm=OpenHRP.StabilizerService.TPCC
        hcf.st_svc.setParameter(stp)
        hcf.startStabilizer ()
        #hcf.abc_svc.goPos(0.5, 0.1, 10)
        #hcf.abc_svc.waitFootSteps()
        hcf.stopStabilizer ()
        # Wait for non-st osscilation being samall
        hcf.seq_svc.setJointAngles(initial_pose, 2.0)
        hcf.waitInterpolation()
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
        # Wait for non-st osscilation being samall
        hcf.seq_svc.setJointAngles(initial_pose, 2.0)
        hcf.waitInterpolation()
        ret = checkActualBaseAttitude()
        if ret:
            print >> sys.stderr, "  Start and Stop Stabilizer => OK"
        assert(ret)
    else:
        print >> sys.stderr, "  This sample is neglected in High-gain mode simulation"

def demoSTLoadPattern ():
    print >> sys.stderr, "5. EEFMQP st + SequencePlayer loadPattern"
    if hcf.pdc:
        stp = hcf.st_svc.getParameter()
        stp.st_algorithm=OpenHRP.StabilizerService.EEFMQP
        stp.emergency_check_mode=OpenHRP.StabilizerService.NO_CHECK # Disable checking of emergency error because currently this error checker does not work correctly during walking.
        hcf.st_svc.setParameter(stp)
        hcf.stopAutoBalancer()
        # Set initial pose of samplerobot-gopos000 before starting of ST
        hcf.seq_svc.setJointAnglesSequenceFull([[0.000242, -0.403476, -0.000185, 0.832071, -0.427767, -6.928952e-05, 0.31129, -0.159481, -0.115399, -0.636277, 0.0, 0.0, 0.0, 0.000242, -0.403469, -0.000185, 0.832073, -0.427775, -6.928781e-05, 0.31129, 0.159481, 0.115399, -0.636277, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]], # jvss
                                               [[0]*29], # vels
                                               [[0]*29], # torques
                                               [[-0.014759, -4.336272e-05, 0.668138]], # poss
                                               [[-0.000245, -0.000862, 0.000171]], # rpys
                                               [[0]*3], # accs
                                               [[0.014052, 0.000203, -0.66798]], # zmps
                                               [[0]*6*4], # wrenchs
                                               [[1,1,0,0,1,1,1,1]], # optionals
                                               [2.0]); # tms
        hcf.seq_svc.waitInterpolation()
        hcf.startStabilizer ()
        # Exec loadPattern
        HRPSYS_DIR=check_output(['pkg-config', 'hrpsys-base', '--variable=prefix']).rstrip()
        hcf.loadPattern(HRPSYS_DIR+'/share/hrpsys/samples/SampleRobot/data/samplerobot-gopos000', 0.0)
        hcf.waitInterpolation()
        hcf.stopStabilizer ()
        # Wait for non-st osscilation being samall
        hcf.seq_svc.setJointAngles(initial_pose, 2.0)
        hcf.waitInterpolation()
        ret = checkActualBaseAttitude()
        if ret:
            print >> sys.stderr, "  ST + loadPattern => OK"
        assert(ret)
    else:
        print >> sys.stderr, "  This sample is neglected in High-gain mode simulation"

def demoSTTurnWalk ():
    print >> sys.stderr, "6. EEFMQP st + Turn walk"
    if hcf.pdc:
        stp = hcf.st_svc.getParameter()
        stp.st_algorithm=OpenHRP.StabilizerService.EEFMQP
        hcf.st_svc.setParameter(stp)
        hcf.startAutoBalancer()
        ggp = hcf.abc_svc.getGaitGeneratorParam()[1]
        org_ggp = hcf.abc_svc.getGaitGeneratorParam()[1]
        ggp.stride_parameter=[0.15, 0.15, 90.0, 0.05]
        hcf.abc_svc.setGaitGeneratorParam(ggp)
        hcf.co_svc.disableCollisionDetection()
        hcf.startStabilizer ()
        hcf.abc_svc.goPos(0,0,175);
        hcf.abc_svc.waitFootSteps();
        hcf.abc_svc.goPos(0.4,0.15,40);
        hcf.abc_svc.waitFootSteps();
        hcf.stopStabilizer ()
        # Wait for non-st osscilation being samalpl
        hcf.abc_svc.setGaitGeneratorParam(org_ggp)
        hcf.co_svc.enableCollisionDetection()
        ret = checkActualBaseAttitude()
        if ret:
            print >> sys.stderr, "  ST + Turnwalk => OK"
        assert(ret)
    else:
        print >> sys.stderr, "  This sample is neglected in High-gain mode simulation"

def demo():
    OPENHRP3_DIR=check_output(['pkg-config', 'openhrp3.1', '--variable=prefix']).rstrip()
    if os.path.exists(OPENHRP3_DIR+"/share/OpenHRP-3.1/sample/model/sample1_bush.wrl"):
        init()
        if hrpsys_version >= '315.5.0':
            demoGetParameter()
            demoSetParameter()
            demoStartStopTPCCST()
            demoStartStopEEFMQPST()
            demoSTLoadPattern()
            demoSTTurnWalk()
    else:
        print >> sys.stderr, "Skip st test because of missing sample1_bush.wrl"

if __name__ == '__main__':
    demo()

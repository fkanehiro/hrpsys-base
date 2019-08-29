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
from distutils.version import StrictVersion

def init ():
    global hcf, initial_pose, half_sitting_pose, hrpsys_version
    hcf = HrpsysConfigurator()
    hcf.getRTCList = hcf.getRTCListUnstable
    hcf.init ("SampleRobot(Robot)0", "$(PROJECT_DIR)/../model/sample1.wrl")
    # set initial pose from sample/controller/SampleController/etc/Sample.pos
    initial_pose = [-7.779e-005,  -0.378613,  -0.000209793,  0.832038,  -0.452564,  0.000244781,  0.31129,  -0.159481,  -0.115399,  -0.636277,  0,  0,  0,  -7.77902e-005,  -0.378613,  -0.000209794,  0.832038,  -0.452564,  0.000244781,  0.31129,  0.159481,  0.115399,  -0.636277,  0,  0,  0,  0,  0,  0]
    half_sitting_pose = [-0.000158,-0.570987,-0.000232,1.26437,-0.692521,0.000277,0.31129,-0.159481,-0.115399,-0.636277,0.0,0.0,0.0,-0.000158,-0.570987,-0.000232,1.26437,-0.692521,0.000277,0.31129,0.159481,0.115399,-0.636277,0.0,0.0,0.0,0.0,0.0,0.0]
    hrpsys_version = hcf.seq.ref.get_component_profile().version
    print("hrpsys_version = %s"%hrpsys_version)
    if StrictVersion(hrpsys_version) >= StrictVersion('315.5.0'):
        # on < 315.5.0 this outputs huge error log message
        hcf.seq_svc.setJointAngles(initial_pose, 2.0)
        hcf.seq_svc.waitInterpolation()
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
    stp_org.eefm_swing_rot_damping_gain = stp_org.eefm_rot_damping_gain[0]
    stp_org.eefm_swing_pos_damping_gain = stp_org.eefm_pos_damping_gain[0]
    stp_org.eefm_use_swing_damping=True
    hcf.st_svc.setParameter(stp_org)
    stp = hcf.st_svc.getParameter()
    vcheck = stp.k_tpcc_p == stp_org.k_tpcc_p and stp.k_tpcc_x == stp_org.k_tpcc_x and stp.k_brot_p == stp_org.k_brot_p
    if vcheck:
        print >> sys.stderr, "  setParameter() => OK", vcheck
    assert(vcheck)

def changeContactDecisionThre (thre):
    stp = hcf.st_svc.getParameter()
    stp.contact_decision_threshold=thre
    hcf.st_svc.setParameter(stp)

def mimicInTheAir ():
    changeContactDecisionThre(10000) # [N]

def mimicOnTheGround ():
    changeContactDecisionThre(50) # [N], default

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

def printActualBase():
    '''Print actual base pos and rot
    '''
    act_base = checkParameterFromLog("WAIST")
    print >> sys.stderr, "  actual base pos = ", act_base[0:3], "[m], actual base rpy = ", act_base[3:], "[rad]"

def changeSTAlgorithm (new_st_alg):
    stp = hcf.st_svc.getParameter()
    if stp.st_algorithm != new_st_alg:
        hcf.stopStabilizer()
        stp.st_algorithm = new_st_alg
        hcf.st_svc.setParameter(stp)
        hcf.startStabilizer ()
        # Wait for osscilation being samall
        hcf.setJointAngles(hcf.getJointAngles(), 2.0);
        hcf.waitInterpolation()

def demoSTLoadPattern ():
    print >> sys.stderr, "3. EEFMQP st + SequencePlayer loadPattern"
    if hcf.pdc:
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
                                               [0.5]); # tms
        hcf.stopAutoBalancer()
        hcf.seq_svc.waitInterpolation()
        stp = hcf.st_svc.getParameter()
        stp.emergency_check_mode=OpenHRP.StabilizerService.NO_CHECK # Disable checking of emergency error because currently this error checker does not work correctly during walking.
        hcf.st_svc.setParameter(stp)
        #changeSTAlgorithm (OpenHRP.StabilizerService.EEFMQPCOP)
        changeSTAlgorithm (OpenHRP.StabilizerService.EEFMQP)
        hcf.startStabilizer()
        # Exec loadPattern
        HRPSYS_DIR=check_output(['pkg-config', 'hrpsys-base', '--variable=prefix']).rstrip()
        hcf.loadPattern(HRPSYS_DIR+'/share/hrpsys/samples/SampleRobot/data/samplerobot-gopos000', 0.0)
        hcf.waitInterpolation()
        ret = checkActualBaseAttitude()
        if ret:
            print >> sys.stderr, "  ST + loadPattern => OK"
        assert(ret)
    else:
        print >> sys.stderr, "  This sample is neglected in High-gain mode simulation"

def demoStartStopTPCCST ():
    print >> sys.stderr, "4. start and stop TPCC st"
    if hcf.pdc:
        # setup controllers
        hcf.startAutoBalancer()
        changeSTAlgorithm (OpenHRP.StabilizerService.TPCC)
        hcf.startStabilizer ()
        #hcf.abc_svc.goPos(0.5, 0.1, 10)
        #hcf.abc_svc.waitFootSteps()
        ret = checkActualBaseAttitude()
        if ret:
            print >> sys.stderr, "  Start and Stop Stabilizer => OK"
        assert(ret)
    else:
        print >> sys.stderr, "  This sample is neglected in High-gain mode simulation"

def demoStartStopEEFMQPST ():
    print >> sys.stderr, "5. start and stop EEFMQP st"
    if hcf.pdc:
        # setup controllers
        hcf.startAutoBalancer()
        changeSTAlgorithm (OpenHRP.StabilizerService.EEFMQP)
        hcf.startStabilizer()
        hcf.abc_svc.goPos(0.3, 0, 0)
        hcf.abc_svc.waitFootSteps()
        ret = checkActualBaseAttitude()
        if ret:
            print >> sys.stderr, "  Start and Stop Stabilizer => OK"
        assert(ret)
    else:
        print >> sys.stderr, "  This sample is neglected in High-gain mode simulation"

def demoSTStairWalk ():
    print >> sys.stderr, "6. EEFMQPCOP + stair"
    if hcf.pdc:
        # setup controllers
        printActualBase()
        changeSTAlgorithm (OpenHRP.StabilizerService.EEFMQPCOP)
        hcf.startStabilizer()
        hcf.startAutoBalancer()
        hcf.seq_svc.setJointAngles(half_sitting_pose, 1.0);
        hcf.waitInterpolation();
        printActualBase()
        # set gg param
        ggp = hcf.abc_svc.getGaitGeneratorParam()[1]
        org_ggp = hcf.abc_svc.getGaitGeneratorParam()[1]
        ggp.default_orbit_type = OpenHRP.AutoBalancerService.STAIR
        ggp.swing_trajectory_time_offset_xy2z=0.1
        ggp.swing_trajectory_delay_time_offset=0.2
        ggp.toe_heel_phase_ratio=[0.05, 0.25, 0.20, 0.0, 0.18, 0.23, 0.09]
        ggp.toe_pos_offset_x = 1e-3*182.0;
        ggp.heel_pos_offset_x = 1e-3*-72.0;
        ggp.toe_zmp_offset_x = 1e-3*182.0;
        ggp.heel_zmp_offset_x = 1e-3*-72.0;
        ggp.use_toe_heel_transition=True
        ggp.use_toe_heel_auto_set = True
        ggp.toe_angle = 20;
        ggp.heel_angle = 10;
        hcf.abc_svc.setGaitGeneratorParam(ggp)
        printActualBase()
        hcf.setFootSteps([OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0,-0.09,0], [1,0,0,0], "rleg")]),
                          OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0.27,0.09,0.1], [1,0,0,0], "lleg")]),
                          OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0.27,-0.09,0.1], [1,0,0,0], "rleg")]),
                          OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0.54,0.09,0], [1,0,0,0], "lleg")]),
                          OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0.54,-0.09,0], [1,0,0,0], "rleg")])]);
        printActualBase()
        hcf.abc_svc.waitFootSteps();
        printActualBase()
        # finish
        hcf.abc_svc.setGaitGeneratorParam(org_ggp)
        ret = checkActualBaseAttitude()
        printActualBase()
        if ret:
            print >> sys.stderr, "  ST + Stair => OK"
        assert(ret)
    else:
        print >> sys.stderr, "  This sample is neglected in High-gain mode simulation"

def demoSTToeHeelWalk ():
    print >> sys.stderr, "7. EEFMQPCOP + toeheel"
    if hcf.pdc:
        # setup controllers
        hcf.startAutoBalancer()
        hcf.co_svc.disableCollisionDetection()
        changeSTAlgorithm (OpenHRP.StabilizerService.EEFMQPCOP)
        hcf.startStabilizer()
        hcf.seq_svc.setJointAngles(initial_pose, 2.0);
        hcf.waitInterpolation();
        # set gg param
        ggp = hcf.abc_svc.getGaitGeneratorParam()[1]
        org_ggp = hcf.abc_svc.getGaitGeneratorParam()[1]
        ggp.default_orbit_type = OpenHRP.AutoBalancerService.RECTANGLE
        ggp.swing_trajectory_time_offset_xy2z=0.1
        ggp.swing_trajectory_delay_time_offset=0.2
        ggp.toe_heel_phase_ratio=[0.05, 0.35, 0.20, 0.0, 0.13, 0.13, 0.14]
        ggp.toe_pos_offset_x = 1e-3*182.0;
        ggp.heel_pos_offset_x = 1e-3*-72.0;
        ggp.toe_zmp_offset_x = 1e-3*182.0;
        ggp.heel_zmp_offset_x = 1e-3*-72.0;
        ggp.use_toe_heel_transition=True
        ggp.use_toe_heel_auto_set=True
        # test setFootStepsWithParam
        ggp.default_double_support_ratio=0.7
        hcf.abc_svc.setGaitGeneratorParam(ggp)
        for sgn in [1, -1]:
            hcf.setFootStepsWithParam([OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0,-0.09,0], [1,0,0,0], "rleg")]),
                                       OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([sgn*0.22,0.09,0], [1,0,0,0], "lleg")]),
                                       OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([sgn*0.44,-0.09,0], [1,0,0,0], "rleg")]),
                                       OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([sgn*0.44,0.09,0], [1,0,0,0], "lleg")])],
                                      [OpenHRP.AutoBalancerService.StepParams([OpenHRP.AutoBalancerService.StepParam(step_height=0.0, step_time=1.0, toe_angle=0.0, heel_angle=0.0)]),
                                       OpenHRP.AutoBalancerService.StepParams([OpenHRP.AutoBalancerService.StepParam(step_height=0.05, step_time=4.0, toe_angle=20.0, heel_angle=10.0)]),
                                       OpenHRP.AutoBalancerService.StepParams([OpenHRP.AutoBalancerService.StepParam(step_height=0.05, step_time=4.0, toe_angle=20.0, heel_angle=10.0)]),
                                       OpenHRP.AutoBalancerService.StepParams([OpenHRP.AutoBalancerService.StepParam(step_height=0.05, step_time=4.0, toe_angle=20.0, heel_angle=10.0)])])
            hcf.abc_svc.waitFootSteps();
        # test goPos
        ggp.default_double_support_ratio=0.2
        ggp.stride_parameter=[0.22,0.1,20.0,0.22]
        ggp.toe_angle = 20;
        ggp.heel_angle = 10;
        hcf.abc_svc.setGaitGeneratorParam(ggp)
        for sgn in [1, -1]:
            hcf.abc_svc.goPos(sgn*0.66,sgn*0.2,sgn*40);
            hcf.abc_svc.waitFootSteps();
        # finish
        hcf.abc_svc.setGaitGeneratorParam(org_ggp)
        hcf.co_svc.enableCollisionDetection()
        ret = checkActualBaseAttitude()
        if ret:
            print >> sys.stderr, "  ST + ToeHeel => OK"
        assert(ret)
    else:
        print >> sys.stderr, "  This sample is neglected in High-gain mode simulation"

def demoSTTurnWalk ():
    print >> sys.stderr, "8. EEFMQPCOP st + Turn walk"
    if hcf.pdc:
        hcf.startAutoBalancer()
        hcf.co_svc.disableCollisionDetection()
        changeSTAlgorithm (OpenHRP.StabilizerService.EEFMQPCOP)
        hcf.startStabilizer()
        ggp = hcf.abc_svc.getGaitGeneratorParam()[1]
        org_ggp = hcf.abc_svc.getGaitGeneratorParam()[1]
        ggp.stride_parameter=[0.15, 0.15, 90.0, 0.05]
        hcf.abc_svc.setGaitGeneratorParam(ggp)
        hcf.abc_svc.goPos(0,-0.2,0);
        hcf.abc_svc.waitFootSteps();
        hcf.abc_svc.goPos(0,0,175);
        hcf.abc_svc.waitFootSteps();
        hcf.abc_svc.goPos(0.4,0.15,40);
        hcf.abc_svc.waitFootSteps();
        # Wait for non-st osscilation being samalpl
        hcf.abc_svc.setGaitGeneratorParam(org_ggp)
        hcf.co_svc.enableCollisionDetection()
        ret = checkActualBaseAttitude()
        if ret:
            print >> sys.stderr, "  ST + Turnwalk => OK"
        assert(ret)
    else:
        print >> sys.stderr, "  This sample is neglected in High-gain mode simulation"


def demoSTTransitionAirGround ():
    # This example is from YoheiKakiuchi's comment : https://github.com/fkanehiro/hrpsys-base/issues/1098, https://github.com/fkanehiro/hrpsys-base/pull/1102#issuecomment-284609203
    print >> sys.stderr, "9. ST Transition (in the air and on the ground)"
    if hcf.pdc:
        # Init
        stp_org = hcf.st_svc.getParameter()
        stp = hcf.st_svc.getParameter()
        stp.transition_time = 0.1; # for fast checking
        hcf.st_svc.setParameter(stp)
        # Tests
        print >> sys.stderr, "  9-1. Check in the air"
        hcf.startStabilizer()
        mimicInTheAir()
        hcf.setJointAngles(hcf.getJointAngles(), stp.transition_time);hcf.waitInterpolation() # Wait transition
        cmode1 = hcf.st_svc.getParameter().controller_mode
        vcheck1 = (cmode1 == OpenHRP.StabilizerService.MODE_AIR)
        print >> sys.stderr, "  9-2. Check on the ground"
        mimicOnTheGround()
        hcf.setJointAngles(hcf.getJointAngles(), stp.transition_time);hcf.waitInterpolation() # Wait transition
        cmode2 = hcf.st_svc.getParameter().controller_mode
        vcheck2 = (cmode2 == OpenHRP.StabilizerService.MODE_ST)
        print >> sys.stderr, "  9-3. Check in the air and then stopST"
        mimicInTheAir()
        hcf.setJointAngles(hcf.getJointAngles(), 0.01);hcf.waitInterpolation() # Wait until in the air flag is invoked in onExecute
        hcf.stopStabilizer()
        cmode3 = hcf.st_svc.getParameter().controller_mode
        vcheck3 = (cmode3 == OpenHRP.StabilizerService.MODE_IDLE)
        print >> sys.stderr, "  9-4. Check on the ground"
        mimicOnTheGround()
        hcf.setJointAngles(hcf.getJointAngles(), 0.01);hcf.waitInterpolation() # Wait until on the ground flag is invoked in onExecute
        hcf.startStabilizer()
        cmode4 = hcf.st_svc.getParameter().controller_mode
        vcheck4 = (cmode4 == OpenHRP.StabilizerService.MODE_ST)
        # Finsh
        hcf.st_svc.setParameter(stp_org)
        vcheck_list = [vcheck1, vcheck2, vcheck3, vcheck4]
        print >> sys.stderr, "  ST Transition Air Ground vcheck = ", vcheck_list, ", cmode = ", [cmode1, cmode2, cmode3, cmode4]
        if all(vcheck_list):
            print >> sys.stderr, "  ST Transition Air Ground => OK"
        assert(all(vcheck_list))
    else:
        print >> sys.stderr, "  This sample is neglected in High-gain mode simulation"

def demoSTRootRotChange ():
    print >> sys.stderr, "10. ST root rot change"
    if hcf.pdc:
        # 10deg
        root_rot_x_pose=[-0.240857,-0.634561,0.012382,1.30211,-0.669201,0.073893,0.31129,-0.159481,-0.115399,-0.636277,0.0,0.0,0.0,-0.232865,-0.555515,0.011753,1.1356,-0.581653,0.06476,0.31129,0.159481,0.115399,-0.636277,0.0,0.0,0.0,0.0,0.0,0.0]
        # 35deg
        root_rot_y_pose=[-1.706033e-05,-1.04708,-0.000479,0.497763,-0.060719,-0.000105,0.31129,-0.159481,-0.115399,-0.636277,0.0,0.0,0.0,-1.690260e-05,-1.04693,-0.000479,0.497318,-0.060422,-0.000105,0.31129,0.159481,0.115399,-0.636277,0.0,0.0,0.0,0.0,0.0,0.0]
        # 25deg
        root_rot_z_pose=[-0.261382,-0.479591,-0.490714,1.26471,-0.722778,0.018041,0.31129,-0.159481,-0.115399,-0.636277,0.0,0.0,0.0,-0.313108,-0.610397,-0.535653,1.24943,-0.571839,-0.013257,0.31129,0.159481,0.115399,-0.636277,0.0,0.0,0.0,0.0,0.0,0.0]
        # all 10deg
        root_rot_xyz_pose=[-0.378611,-0.81283,-0.238181,1.23534,-0.577915,0.061071,0.31129,-0.159481,-0.115399,-0.636277,0.0,0.0,0.0,-0.351695,-0.768514,-0.225097,1.05221,-0.442267,0.050849,0.31129,0.159481,0.115399,-0.636277,0.0,0.0,0.0,0.0,0.0,0.0]
        hcf.startAutoBalancer();
        changeSTAlgorithm (OpenHRP.StabilizerService.EEFMQPCOP)
        print >> sys.stderr, "  init"
        checkActualBaseAttitude()
        hcf.seq_svc.setJointAngles(root_rot_x_pose, 1.0);hcf.waitInterpolation();
        hcf.seq_svc.setJointAngles(root_rot_x_pose, 1.0);hcf.waitInterpolation(); # dummy for wait
        print >> sys.stderr, "  root rot x done."
        checkActualBaseAttitude()
        hcf.seq_svc.setJointAngles(root_rot_y_pose, 1.0);hcf.waitInterpolation();
        hcf.seq_svc.setJointAngles(root_rot_y_pose, 1.0);hcf.waitInterpolation(); # dummy for wait
        print >> sys.stderr, "  root rot y done."
        checkActualBaseAttitude()
        hcf.seq_svc.setJointAngles(root_rot_z_pose, 1.0);hcf.waitInterpolation();
        hcf.seq_svc.setJointAngles(root_rot_z_pose, 1.0);hcf.waitInterpolation(); # dummy for wait
        print >> sys.stderr, "  root rot z done."
        checkActualBaseAttitude()
        hcf.seq_svc.setJointAngles(root_rot_xyz_pose, 1.0);hcf.waitInterpolation();
        hcf.seq_svc.setJointAngles(root_rot_xyz_pose, 1.0);hcf.waitInterpolation(); # dummy for wait
        hcf.seq_svc.setJointAngles(initial_pose, 1.0);hcf.waitInterpolation();
        print >> sys.stderr, "  root rot xyz done."
        ret = checkActualBaseAttitude()
        if ret:
            print >> sys.stderr, "  ST root rot change => OK"
        assert(ret)
    else:
        print >> sys.stderr, "  This sample is neglected in High-gain mode simulation"

def demoSTMimicRouchTerrainWalk (terrain_height_diff = 0.04):
    print >> sys.stderr, "11. ST mimic rough terrain walk"
    if hcf.pdc:
        hcf.setFootSteps([OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0,-0.09,0], [1,0,0,0], "rleg")]),
                          OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0.22,0.09,terrain_height_diff], [1,0,0,0], "lleg")]),
                          OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0.44,-0.09,0], [1,0,0,0], "rleg")]),
                          OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0.44,0.09,0], [1,0,0,0], "lleg")])]);
        hcf.abc_svc.waitFootSteps();
        hcf.setFootSteps([OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0,-0.09,0], [1,0,0,0], "rleg")]),
                          OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0.22,0.09,-1*terrain_height_diff], [1,0,0,0], "lleg")]),
                          OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0.44,-0.09,0], [1,0,0,0], "rleg")]),
                          OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0.44,0.09,0], [1,0,0,0], "lleg")])]);
        hcf.abc_svc.waitFootSteps();
        ret = checkActualBaseAttitude()
        if ret:
            print >> sys.stderr, "  ST mimic rough terrain walk => OK"
        assert(ret)
    else:
        print >> sys.stderr, "  This sample is neglected in High-gain mode simulation"


def demo():
    OPENHRP3_DIR=check_output(['pkg-config', 'openhrp3.1', '--variable=prefix']).rstrip()
    if os.path.exists(OPENHRP3_DIR+"/share/OpenHRP-3.1/sample/model/sample1_bush.wrl"):
        init()
        if StrictVersion(hrpsys_version) >= StrictVersion('315.5.0'):
            demoGetParameter()
            demoSetParameter()
            demoSTLoadPattern()
            demoStartStopTPCCST()
            demoStartStopEEFMQPST()
            demoSTStairWalk()
            demoSTToeHeelWalk()
            demoSTTurnWalk()
            demoSTTransitionAirGround()
            demoSTRootRotChange()
            demoSTMimicRouchTerrainWalk()
    else:
        print >> sys.stderr, "Skip st test because of missing sample1_bush.wrl"

if __name__ == '__main__':
    demo()

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

from subprocess import check_output

# set parameter
def demoSetParameter():
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
    stp_org.st_algorithm=OpenHRP.StabilizerService.EEFMQP
    stp_org.is_ik_enable=[True]*4
    hcf.st_svc.setParameter(stp_org)

def init ():
    global hcf, initial_pose, dualarm_via_pose, dualarm_reach_pose, dualarm_liftup_pose, singlearm_via_pose, singlearm_reach_pose, singlearm_liftup_pose, dualarm_push_pose
    hcf = HrpsysConfigurator()
    hcf.getRTCList = hcf.getRTCListUnstable
    hcf.init ("SampleRobot(Robot)0", "$(PROJECT_DIR)/../model/sample1.wrl")
    # Pose setting
    initial_pose=[-7.778932e-05,-0.378613,-0.00021,0.832039,-0.452564,0.000245,0.993363,-0.165936,-0.232906,-1.64155,-0.009413,-0.362925,0.0,-7.778932e-05,-0.378613,-0.00021,0.832039,-0.452564,0.000245,0.993407,0.16582,0.232707,-1.64166,0.009397,-0.362966,0.0,0.0,0.0,0.0]
    dualarm_via_pose=[-1.301383e-16,-0.4932,-4.784822e-17,1.26321,-0.737682,1.436666e-16,0.584706,-0.143933,0.086868,-1.76694,-1.69575,-0.112369,0.637045,-2.103971e-16,-0.4932,-3.815624e-17,1.26321,-0.737682,2.133439e-16,0.584706,0.143933,-0.086868,-1.76694,1.69575,-0.112369,-0.637045,-0.369579,7.202322e-16,1.814631e-16]
    dualarm_reach_pose=[1.552547e-16,-0.531004,7.070416e-17,1.21764,-0.67113,-9.670336e-17,-0.049061,-0.07576,0.074177,-1.18878,-1.66902,-0.040133,0.637045,5.975712e-17,-0.531004,6.854620e-17,1.21764,-0.67113,-1.725813e-17,-0.049061,0.07576,-0.074177,-1.18878,1.66902,-0.040133,-0.637045,-0.322025,-1.568538e-16,-4.382552e-16]
    dualarm_liftup_pose=[4.954412e-15,-0.524931,8.930202e-15,1.07007,-0.525833,-3.445727e-16,-0.16089,-0.406321,0.424654,-1.57558,-2.01019,-0.390167,0.637045,4.790551e-15,-0.524931,8.955748e-15,1.07007,-0.525833,-1.750785e-16,-0.16089,0.406321,-0.424654,-1.57558,2.01019,-0.390167,-0.637045,0.007999,4.191514e-16,-4.457395e-14]
    singlearm_via_pose = [-0.072674,-0.322265,0.02671,0.793691,-0.449511,0.091832,0.930492,-0.172216,0.271035,-2.15462,-0.94786,-0.274689,0.637045,-0.072879,-0.32137,0.026706,0.807374,-0.464093,0.092013,0.31129,0.159481,0.115399,-0.636277,0.0,0.0,0.0,-0.210667,0.569051,-0.203392]
    singlearm_reach_pose = [-0.065582,-0.348364,-0.036375,0.770359,-0.384613,0.05823,0.267732,0.067753,-0.2238,-1.88388,-1.20653,-0.157041,0.637045,-0.066042,-0.362442,-0.036569,0.777637,-0.377806,0.05814,0.31129,0.159481,0.115399,-0.636277,0.0,0.0,0.0,0.119774,0.357853,0.377041]
    singlearm_liftup_pose = [-0.043429,-0.330743,-0.038834,0.732611,-0.37014,0.028879,0.256358,0.163926,-0.247853,-1.90506,-1.28971,-0.110907,0.637045,-0.043613,-0.34042,-0.038972,0.729501,-0.357353,0.028662,0.31129,0.159481,0.115399,-0.636277,0.0,0.0,0.0,0.089304,0.185283,0.369292]
    dualarm_push_pose=[-3.998549e-05,-0.710564,-0.000264,1.41027,-0.680767,-2.335251e-05,-0.541944,-0.091558,0.122667,-1.02616,-1.71287,-0.056837,1.5708,-3.996804e-05,-0.710511,-0.000264,1.41016,-0.680706,-2.333505e-05,-0.542,0.091393,-0.122578,-1.02608,1.71267,-0.05677,-1.5708,0.006809,0.000101,-0.000163]
    hcf.seq_svc.setJointAngles(initial_pose, 2.0)
    hcf.waitInterpolation()
    # Initialize controllers
    hcf.co_svc.disableCollisionDetection()
    ggp=hcf.abc_svc.getGaitGeneratorParam()[1];
    ggp.stride_parameter=[0.15, 0.08, 25.0, 0.15];
    hcf.abc_svc.setGaitGeneratorParam(ggp);
    demoSetParameter()
    icp=hcf.ic_svc.getImpedanceControllerParam("rarm")[1]
    #icp.K_p = 400
    icp.D_p = 400
    icp.K_r = 1e5
    icp.D_r = 1e5
    hcf.ic_svc.setImpedanceControllerParam("rarm", icp)
    hcf.ic_svc.setImpedanceControllerParam("larm", icp)
    hcf.startDefaultUnstableControllers(['rarm', 'larm'], ["rleg", "lleg", "rarm", "larm"])
    HRPSYS_DIR=check_output(['pkg-config', 'hrpsys-base', '--variable=prefix']).rstrip()
    hcf.rmfo_svc.loadForceMomentOffsetParams(HRPSYS_DIR+'/share/hrpsys/samples/SampleRobot/ForceSensorOffset_SampleRobot.txt')

def demoDualarmCarryup (is_walk=True, auto_detecion = True):
    print >> sys.stderr, "1. Dual-arm carry-up demo."
    print >> sys.stderr, "  Reaching"
    hcf.seq_svc.setJointAngles(dualarm_via_pose, 2.0)
    hcf.waitInterpolation()
    hcf.seq_svc.setJointAngles(dualarm_reach_pose, 2.0)
    hcf.waitInterpolation()
    print >> sys.stderr, "  Increase operational force"
    if auto_detecion:
        objectTurnaroundDetection()
    else:
        hcf.seq_svc.setWrenches([0]*6+[0]*6+[0,0,-9.8*2.5,0,0,0]*2, 2.0) # 2.5[kg]*2 = 5.0[kg]
        hcf.waitInterpolation()
    print >> sys.stderr, "  Lift up & down"
    hcf.seq_svc.setJointAngles(dualarm_liftup_pose, 2.0)
    hcf.waitInterpolation()
    if is_walk:
        demoWalk()
    hcf.seq_svc.setJointAngles(dualarm_reach_pose, 2.0)
    hcf.waitInterpolation()
    print >> sys.stderr, "  Reset operational force"
    hcf.seq_svc.setWrenches([0]*24, 2.0)
    hcf.waitInterpolation()
    print >> sys.stderr, "  Releasing"
    hcf.seq_svc.setJointAngles(dualarm_via_pose, 2.0)
    hcf.waitInterpolation()
    hcf.seq_svc.setJointAngles(initial_pose, 2.0)
    hcf.waitInterpolation()

def demoSinglearmCarryup (is_walk=True, auto_detecion = True):
    print >> sys.stderr, "2. Single-arm carry-up demo."
    print >> sys.stderr, "  Reaching"
    hcf.abc_svc.goPos(0.02,0.15,0)
    hcf.abc_svc.waitFootSteps();
    hcf.seq_svc.setJointAngles(singlearm_via_pose, 2.0)
    hcf.waitInterpolation()
    hcf.seq_svc.setJointAngles(singlearm_reach_pose, 2.0)
    hcf.waitInterpolation()
    print >> sys.stderr, "  Increase operational force"
    if auto_detecion:
        objectTurnaroundDetection(limbs=['rarm'])
    else:
        hcf.seq_svc.setWrenches([0]*6+[0]*6+[0]*6+[0,0,-9.8*5.0,0,0,0], 2.0)
        hcf.waitInterpolation()
    print >> sys.stderr, "  Lift up & down"
    hcf.seq_svc.setJointAngles(singlearm_liftup_pose, 2.0)
    hcf.waitInterpolation()
    if is_walk:
        hcf.setJointAngle("RARM_WRIST_R", 10, 0.3)
        hcf.waitInterpolation()
        demoWalk()
    hcf.seq_svc.setJointAngles(singlearm_reach_pose, 2.0)
    hcf.waitInterpolation()
    print >> sys.stderr, "  Reset operational force"
    hcf.seq_svc.setWrenches([0]*24, 2.0)
    hcf.waitInterpolation()
    print >> sys.stderr, "  Releasing"
    hcf.seq_svc.setJointAngles(singlearm_via_pose, 2.0)
    hcf.waitInterpolation()
    hcf.seq_svc.setJointAngles(initial_pose, 2.0)
    hcf.waitInterpolation()

def objectTurnaroundDetection(max_time = 4.0, max_ref_force = 9.8*6.0, limbs=["rarm", "larm"], axis=[0,0,-1]):
    otdp=hcf.ic_svc.getObjectTurnaroundDetectorParam()[1]
    otdp.detect_time_thre=0.3
    otdp.start_time_thre=0.3
    otdp.axis=axis
    hcf.ic_svc.setObjectTurnaroundDetectorParam(otdp)
    if limbs==['rarm']:
        force = [axis[0]*max_ref_force, axis[1]*max_ref_force, axis[2]*max_ref_force]
        hcf.seq_svc.setWrenches([0]*18+force+[0,0,0], max_time)
    else:
        force = [axis[0]*max_ref_force*0.5, axis[1]*max_ref_force*0.5, axis[2]*max_ref_force*0.5]
        hcf.seq_svc.setWrenches([0]*12+force+[0]*3+force+[0]*3, max_time)
    hcf.ic_svc.startObjectTurnaroundDetection(max_ref_force, max_time+2.0, limbs)
    flg = True
    while flg:
        tmpflg = hcf.ic_svc.checkObjectTurnaroundDetection()
        #print rtm.readDataPort(hcf.rmfo.port("off_rhsensor")).data, rtm.readDataPort(hcf.rmfo.port("off_lhsensor")).data
        print "  flag = ", tmpflg, ", forces = ", hcf.ic_svc.getObjectForcesMoments()[1][0], ", moments = ", hcf.ic_svc.getObjectForcesMoments()[2][0]
        flg = (tmpflg == OpenHRP.ImpedanceControllerService.MODE_DETECTOR_IDLE) or (tmpflg == OpenHRP.ImpedanceControllerService.MODE_STARTED)
        time.sleep(0.5)
    print "  flag = ", tmpflg, ", forces = ", hcf.ic_svc.getObjectForcesMoments()[1][0], ", moments = ", hcf.ic_svc.getObjectForcesMoments()[2][0]
    if limbs==['rarm']:
        hcf.seq_svc.setWrenches([0]*18+hcf.ic_svc.getObjectForcesMoments()[1][0]+hcf.ic_svc.getObjectForcesMoments()[2][0], 2.0)
    else:
        hcf.seq_svc.setWrenches([0]*12+hcf.ic_svc.getObjectForcesMoments()[1][0]+hcf.ic_svc.getObjectForcesMoments()[2][0]+hcf.ic_svc.getObjectForcesMoments()[1][1]+hcf.ic_svc.getObjectForcesMoments()[2][1], 2.0)
    hcf.waitInterpolation()

def demoWalk ():
    hcf.abc_svc.goPos(-0.3,-0.1,0);
    hcf.abc_svc.waitFootSteps();
    hcf.abc_svc.goPos(0,0,30);
    hcf.abc_svc.waitFootSteps();
    hcf.abc_svc.goPos(0,0,-30);
    hcf.abc_svc.waitFootSteps();
    hcf.abc_svc.goPos(0.3,0.1,0);
    hcf.abc_svc.waitFootSteps();

def demoDualarmPush (auto_detecion = True):
    print >> sys.stderr, "3. Dual-arm push demo."
    print >> sys.stderr, "  Move to"
    hcf.abc_svc.goPos(-0.45,0,0);
    hcf.abc_svc.waitFootSteps();
    hcf.abc_svc.goPos(0,0,(math.degrees(rtm.readDataPort(rtm.findRTC("PushBox(Robot)0").port("WAIST")).data.orientation.y-rtm.readDataPort(hcf.rh.port("WAIST")).data.orientation.y)));
    hcf.abc_svc.waitFootSteps();
    print >> sys.stderr, "  Reaching"
    #hcf.abc_svc.goPos(0.25, -1*(rtm.readDataPort(rtm.findRTC("PushBox(Robot)0").port("WAIST")).data.position.x-rtm.readDataPort(hcf.rh.port("WAIST")).data.position.x), 0);
    hcf.abc_svc.goPos(0.1, -1*(rtm.readDataPort(rtm.findRTC("PushBox(Robot)0").port("WAIST")).data.position.x-rtm.readDataPort(hcf.rh.port("WAIST")).data.position.x), 0);
    hcf.abc_svc.waitFootSteps();
    hcf.seq_svc.setJointAngles(dualarm_via_pose, 1.0)
    hcf.waitInterpolation()
    hcf.seq_svc.setJointAngles(dualarm_push_pose, 1.0)
    hcf.waitInterpolation()
    print >> sys.stderr, "  Increase operational force"
    if auto_detecion:
        objectTurnaroundDetection(axis=[-1,0,0],max_ref_force=100, max_time=2)
    else:
        hcf.seq_svc.setWrenches([0]*6+[0]*6+[-40,0,0,0,0,0]*2, 2.0)
        hcf.waitInterpolation()
    print >> sys.stderr, "  Push forward"
    abcp=hcf.abc_svc.getAutoBalancerParam()[1]
    abcp.is_hand_fix_mode = True
    hcf.abc_svc.setAutoBalancerParam(abcp)
    hcf.abc_svc.goPos(0.5,0,0)
    hcf.abc_svc.waitFootSteps();
    hcf.seq_svc.setWrenches([0]*24, 2.0)
    hcf.waitInterpolation()

def demo ():
    init()
    demoDualarmCarryup()
    demoSinglearmCarryup()
    demoDualarmPush()

if __name__ == '__main__':
    demo()

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
    hcf.st_svc.setParameter(stp_org)

def init ():
    global hcf, initial_pose, dualarm_via_pose, dualarm_reach_pose, dualarm_liftup_pose, singlearm_via_pose, singlearm_reach_pose, singlearm_liftup_pose
    hcf = HrpsysConfigurator()
    hcf.getRTCList = hcf.getRTCListUnstable
    hcf.init ("SampleRobot(Robot)0", "$(PROJECT_DIR)/../model/sample1.wrl")
    # Pose setting
    initial_pose=[-7.778932e-05,-0.378613,-0.00021,0.832039,-0.452564,0.000245,0.993363,-0.165936,-0.232906,-1.64155,-0.009413,-0.362925,0.0,-7.778932e-05,-0.378613,-0.00021,0.832039,-0.452564,0.000245,0.993407,0.16582,0.232707,-1.64166,0.009397,-0.362966,0.0,0.0,0.0,0.0]
    dualarm_via_pose=[-1.301383e-16,-0.4932,-4.784822e-17,1.26321,-0.737682,1.436666e-16,0.584706,-0.143933,0.086868,-1.76694,-1.69575,-0.112369,0.637045,-2.103971e-16,-0.4932,-3.815624e-17,1.26321,-0.737682,2.133439e-16,0.584706,0.143933,-0.086868,-1.76694,1.69575,-0.112369,-0.637045,-0.369579,7.202322e-16,1.814631e-16]
    dualarm_reach_pose=[1.552547e-16,-0.531004,7.070416e-17,1.21764,-0.67113,-9.670336e-17,-0.049061,-0.07576,0.074177,-1.18878,-1.66902,-0.040133,0.637045,5.975712e-17,-0.531004,6.854620e-17,1.21764,-0.67113,-1.725813e-17,-0.049061,0.07576,-0.074177,-1.18878,1.66902,-0.040133,-0.637045,-0.322025,-1.568538e-16,-4.382552e-16]
    dualarm_liftup_pose=[4.954412e-15,-0.524931,8.930202e-15,1.07007,-0.525833,-3.445727e-16,-0.16089,-0.406321,0.424654,-1.57558,-2.01019,-0.390167,0.637045,4.790551e-15,-0.524931,8.955748e-15,1.07007,-0.525833,-1.750785e-16,-0.16089,0.406321,-0.424654,-1.57558,2.01019,-0.390167,-0.637045,0.007999,4.191514e-16,-4.457395e-14]
    singlearm_via_pose=[-0.068195,-0.335363,0.041187,0.813543,-0.456522,0.108135,0.83455,0.170568,0.507491,-1.69645,-0.425082,-0.50636,0.637045,-0.068541,-0.339426,0.041258,0.847517,-0.48645,0.108662,0.31129,0.159481,0.115399,-0.636277,0.0,0.0,0.0,-0.233088,0.756826,-0.278698]
    singlearm_reach_pose=[-0.040522,-0.379505,-0.014115,0.794016,-0.38533,0.061831,-0.165685,0.261435,0.40262,-1.16008,-0.970295,-0.539249,0.637045,-0.041204,-0.400725,-0.014227,0.8308,-0.400887,0.062193,0.31129,0.159481,0.115399,-0.636277,0.0,0.0,0.0,0.113797,0.471303,0.041281]
    singlearm_liftup_pose=[-0.009622,-0.351505,-0.02216,0.756859,-0.385877,0.020499,-0.107416,0.261774,0.382684,-1.24247,-1.20811,-0.592562,0.637045,-0.01006,-0.370083,-0.022316,0.783906,-0.39434,0.020496,0.31129,0.159481,0.115399,-0.636277,0.0,0.0,0.0,-0.000185,0.221567,0.116591]
    hcf.seq_svc.setJointAngles(initial_pose, 2.0)
    hcf.waitInterpolation()
    # Initialize controllers
    hcf.co_svc.disableCollisionDetection()
    hcf.startAutoBalancer(["rleg", "lleg", "rarm", "larm"])
    stp=hcf.st_svc.getParameter()
    stp.is_ik_enable=[True]*4
    hcf.st_svc.setParameter(stp)
    demoSetParameter()
    hcf.startStabilizer()
    icp=hcf.ic_svc.getImpedanceControllerParam("rarm")[1]
    icp.D_p = 400
    icp.K_r = 1e5
    icp.D_r = 1e5
    hcf.ic_svc.setImpedanceControllerParam("rarm", icp)
    hcf.ic_svc.setImpedanceControllerParam("larm", icp)
    hcf.ic_svc.startImpedanceController("rarm")
    hcf.ic_svc.startImpedanceController("larm")

def demoDualarmCarryup (is_walk=True):
    print >> sys.stderr, "1. Dual-arm carry-up demo."
    print >> sys.stderr, "  Reaching"
    hcf.seq_svc.setJointAngles(dualarm_via_pose, 2.0)
    hcf.waitInterpolation()
    hcf.seq_svc.setJointAngles(dualarm_reach_pose, 2.0)
    hcf.waitInterpolation()
    print >> sys.stderr, "  Increase operational force"
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

def demoSinglearmCarryup (is_walk=True):
    print >> sys.stderr, "2. Single-arm carry-up demo."
    print >> sys.stderr, "  Reaching"
    hcf.seq_svc.setJointAngles(singlearm_via_pose, 2.0)
    hcf.waitInterpolation()
    hcf.seq_svc.setJointAngles(singlearm_reach_pose, 2.0)
    hcf.waitInterpolation()
    print >> sys.stderr, "  Increase operational force"
    hcf.seq_svc.setWrenches([0]*6+[0]*6+[0]*6+[0,0,-9.8*5.0,0,0,0], 2.0)
    hcf.waitInterpolation()
    print >> sys.stderr, "  Lift up & down"
    hcf.seq_svc.setJointAngles(singlearm_liftup_pose, 2.0)
    hcf.waitInterpolation()
    if is_walk:
        hcf.setJointAngle("RARM_WRIST_R", 11, 0.3)
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

def demoWalk ():
    hcf.abc_svc.goPos(-0.3,-0.1,0);
    hcf.abc_svc.waitFootSteps();
    hcf.abc_svc.goPos(0,0,30);
    hcf.abc_svc.waitFootSteps();
    hcf.abc_svc.goPos(0,0,-30);
    hcf.abc_svc.waitFootSteps();
    hcf.abc_svc.goPos(0.3,0.1,0);
    hcf.abc_svc.waitFootSteps();

def demo ():
    init()
    demoDualarmCarryup()
    demoSinglearmCarryup()

if __name__ == '__main__':
    demo()

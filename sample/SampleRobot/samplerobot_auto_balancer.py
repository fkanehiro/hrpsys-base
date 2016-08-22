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

def defJointGroups ():
    rleg_6dof_group = ['rleg', ['RLEG_HIP_R', 'RLEG_HIP_P', 'RLEG_HIP_Y', 'RLEG_KNEE', 'RLEG_ANKLE_P', 'RLEG_ANKLE_R']]
    lleg_6dof_group = ['lleg', ['LLEG_HIP_R', 'LLEG_HIP_P', 'LLEG_HIP_Y', 'LLEG_KNEE', 'LLEG_ANKLE_P', 'LLEG_ANKLE_R']]
    torso_group = ['torso', ['WAIST_P', 'WAIST_R', 'CHEST']]
    head_group = ['head', []]
    rarm_group = ['rarm', ['RARM_SHOULDER_P', 'RARM_SHOULDER_R', 'RARM_SHOULDER_Y', 'RARM_ELBOW', 'RARM_WRIST_Y', 'RARM_WRIST_P', 'RARM_WRIST_R']]
    larm_group = ['larm', ['LARM_SHOULDER_P', 'LARM_SHOULDER_R', 'LARM_SHOULDER_Y', 'LARM_ELBOW', 'LARM_WRIST_Y', 'LARM_WRIST_P', 'LARM_WRIST_R']]
    return [rleg_6dof_group, lleg_6dof_group, torso_group, head_group, rarm_group, larm_group]

def init ():
    global hcf, initial_pose, arm_front_pose, half_sitting_pose, root_rot_x_pose, root_rot_y_pose, pose_list, hrpsys_version, four_legs_mode_pose
    hcf = HrpsysConfigurator()
    hcf.getRTCList = hcf.getRTCListUnstable
    hcf.init ("SampleRobot(Robot)0", "$(PROJECT_DIR)/../model/sample1.wrl")
    hcf.connectLoggerPort(hcf.abc, 'baseRpyOut') # Just for checking
    hcf.Groups = defJointGroups()
    # set initial pose from sample/controller/SampleController/etc/Sample.pos
    initial_pose = [-7.779e-005,  -0.378613,  -0.000209793,  0.832038,  -0.452564,  0.000244781,  0.31129,  -0.159481,  -0.115399,  -0.636277,  0,  0,  0.637045,  -7.77902e-005,  -0.378613,  -0.000209794,  0.832038,  -0.452564,  0.000244781,  0.31129,  0.159481,  0.115399,  -0.636277,  0,  0,  -0.637045,  0,  0,  0]
    arm_front_pose = [-7.778932e-05,-0.378613,-0.00021,0.832039,-0.452564,0.000245,-1.5708,-0.159481,-0.115399,-0.349066,0.0,0.0,0.0,-7.778932e-05,-0.378613,-0.00021,0.832039,-0.452564,0.000245,-1.5708,0.159481,0.115399,-0.349066,0.0,0.0,0.0,0.0,0.0,0.0]
    half_sitting_pose = [-0.000158,-0.570987,-0.000232,1.26437,-0.692521,0.000277,0.31129,-0.159481,-0.115399,-0.636277,0.0,0.0,0.0,-0.000158,-0.570987,-0.000232,1.26437,-0.692521,0.000277,0.31129,0.159481,0.115399,-0.636277,0.0,0.0,0.0,0.0,0.0,0.0]
    root_rot_x_pose = [-0.241557,-0.634167,0.011778,1.30139,-0.668753,0.074236,0.31129,-0.159481,-0.115399,-0.636277,0.0,0.0,0.0,-0.233491,-0.555191,0.011181,1.13468,-0.580942,0.065086,0.31129,0.159481,0.115399,-0.636277,0.0,0.0,0.0,0.0,0.0,0.0]
    root_rot_y_pose = [8.251963e-05,-0.980029,-0.000384,1.02994,-0.398115,-0.000111,0.31129,-0.159481,-0.115399,-0.636277,0.0,0.0,0.0,8.252625e-05,-0.980033,-0.000384,1.02986,-0.398027,-0.000111,0.31129,0.159481,0.115399,-0.636277,0.0,0.0,0.0,0.0,0.0,0.0]
    four_legs_mode_pose = [0.0, -0.349066, 0.0, 0.820305, -0.471239, 0.0, 0.493231, 0.008013, 0.000304, -1.608, 0.008019, -0.456023, 0.637045, 0.0, -0.349066, 0.0, 0.820305, -0.471239, 0.0, 0.493231, -0.008013, -0.000304, -1.608, -0.008019, -0.456023, -0.637045, 0.0, 0.0, 0.0]
    pose_list=[half_sitting_pose, root_rot_x_pose, root_rot_y_pose, arm_front_pose, four_legs_mode_pose]
    hcf.seq_svc.setJointAngles(initial_pose, 2.0)
    hcf.waitInterpolation()
    hrpsys_version = hcf.seq.ref.get_component_profile().version
    print("hrpsys_version = %s"%hrpsys_version)

def testPoseList(pose_list, initial_pose):
    for pose in pose_list:
        hcf.seq_svc.setJointAngles(pose, 1.0)
        hcf.waitInterpolation()
        hcf.seq_svc.setJointAngles(initial_pose, 1.0)
        hcf.waitInterpolation()

def saveLogForCheckParameter(log_fname="/tmp/test-samplerobot-auto-balancer-check-param"):
    hcf.setMaxLogLength(1);hcf.clearLog();time.sleep(0.1);hcf.saveLog(log_fname)

def checkParameterFromLog(port_name, log_fname="/tmp/test-samplerobot-auto-balancer-check-param", save_log=True, rtc_name="SampleRobot(Robot)0"):
    if save_log:
        saveLogForCheckParameter(log_fname)
    return map(float, open(log_fname+"."+rtc_name+"_"+port_name, "r").readline().split(" ")[1:-1])

def checkActualBaseAttitude(ref_rpy = None, thre=0.1): # degree
    '''Check whether the robot falls down based on actual robot base-link attitude.
    '''
    act_rpy = checkParameterFromLog("WAIST")[3:]
    if ref_rpy == None:
        ref_rpy = checkParameterFromLog("baseRpyOut", rtc_name="sh", save_log=False)
    ret = abs(math.degrees(act_rpy[0]-ref_rpy[0])) < thre and abs(math.degrees(act_rpy[1]-ref_rpy[1])) < thre
    print >> sys.stderr, "  ret = ", ret, ", actual base rpy = (", act_rpy, "), ", "reference base rpy = (", ref_rpy, ")"
    assert (ret)
    return ret

def Quaternion2Angle(q):
    w, v = q[0], q[1:]
    theta = math.acos(w) * 2.0
    return theta

def Quaternion2RotMatrixZ(q):
    theta = Quaternion2Angle(q)
    return numpy.array([[numpy.cos(theta), -numpy.sin(theta), 0],
                        [numpy.sin(theta),  numpy.cos(theta), 0],
                        [               0,                 0, 1]])


def calcDiffFootMidCoords (prev_dst_foot_midcoords):
    '''Calculate difference from previous dst_foot_midcoords and current dst_foot_midcoords.
    Returns difx, dify, difth, which are gopos parameters
    '''
    new_dst_foot_midcoords=hcf.abc_svc.getFootstepParam()[1].dst_foot_midcoords
    # Check diff
    difxy = (Quaternion2RotMatrixZ(prev_dst_foot_midcoords.rot).transpose()).dot((numpy.array([new_dst_foot_midcoords.pos])-numpy.array([prev_dst_foot_midcoords.pos])).transpose())
    difth = math.degrees(Quaternion2Angle(new_dst_foot_midcoords.rot)-Quaternion2Angle(prev_dst_foot_midcoords.rot))
    return [difxy[0,0], difxy[1,0], difth]

def checkGoPosParam (goalx, goaly, goalth, prev_dst_foot_midcoords):
    '''Check whether goPos argument are correctly achieved based on dst_foot_midcoords values.
    goPos params should be "new_dst_foot_midcoords - prev_dst_foot_midcoords"
    '''
    # Check diff
    [difx, dify, difth] = calcDiffFootMidCoords(prev_dst_foot_midcoords)
    ret = (abs(difx-goalx) < 1e-5 and abs(dify-goaly) < 1e-5 and abs(difth-goalth) < 1e-5)
    print >> sys.stderr, "  Check goPosParam (diff = ", (difx-goalx), "[m], ", (dify-goaly), "[m], ", (difth-goalth), "[deg])"
    print >> sys.stderr, "  => ", ret
    assert(ret)
    return ret

def calcVelListFromPosList(pos_list, dt):
    '''Calculate velocity list from position list.
    Element of pos_list and vel_list should be list like [0,0,0].
    '''
    vel_list=[]
    ppos=pos_list[0]
    for pos in pos_list:
        vel_list.append(map(lambda x, y: (x-y)/dt, pos, ppos));
        ppos=pos
    return vel_list

def checkTooLargeABCCogAcc (acc_thre = 5.0): # [m/s^2]
    '''Check ABC too large cog acceleration.
    This is used discontinuous cog trajectory.
    '''
    # Parse COG [m] and tm [s]
    cog_list=[]
    tm_list=[]
    for line in open("/tmp/test-abc-log.abc_cogOut", "r"):
        tm_list.append(float(line.split(" ")[0]));
        cog_list.append(map(float, line.split(" ")[1:-1]));
    cog_list=cog_list[:-1000] # ?? Neglect latter elements
    dt = tm_list[1]-tm_list[0] # [s]
    # Calculate velocity and acceleration
    dcog_list=calcVelListFromPosList(cog_list, dt)
    ddcog_list=calcVelListFromPosList(dcog_list, dt)
    # Check max
    max_cogx_acc = max(map(lambda x : abs(x[0]), ddcog_list))
    max_cogy_acc = max(map(lambda x : abs(x[1]), ddcog_list))
    ret = (max_cogx_acc < acc_thre) and (max_cogy_acc < acc_thre)
    print >> sys.stderr, "  Check acc x = ", max_cogx_acc, ", y = ", max_cogy_acc, ", thre = ", acc_thre, "[m/s^2], ret = ", ret
    assert(ret)

def demoAutoBalancerFixFeet ():
    print >> sys.stderr, "1. AutoBalancer mode by fixing feet"
    hcf.startAutoBalancer(["rleg", "lleg"]);
    hcf.seq_svc.setJointAngles(arm_front_pose, 1.0)
    hcf.waitInterpolation()
    hcf.seq_svc.setJointAngles(initial_pose, 1.0)
    hcf.waitInterpolation()
    hcf.stopAutoBalancer();
    checkActualBaseAttitude()
    print >> sys.stderr, "  Start and Stop AutoBalancer by fixing feet=>OK"

def demoAutoBalancerFixFeetHands ():
    print >> sys.stderr, "2. AutoBalancer mode by fixing hands and feet"
    hcf.startAutoBalancer()
    hcf.seq_svc.setJointAngles(arm_front_pose, 1.0)
    hcf.waitInterpolation()
    hcf.seq_svc.setJointAngles(initial_pose, 1.0)
    hcf.waitInterpolation()
    hcf.stopAutoBalancer();
    checkActualBaseAttitude()
    print >> sys.stderr, "  Start and Stop AutoBalancer by fixing hands and feet=>OK"

def demoAutoBalancerGetParam():
    print >> sys.stderr, "3. getAutoBalancerParam"
    ret = hcf.abc_svc.getAutoBalancerParam()
    if ret[0]:
        print >> sys.stderr, "  getAutoBalancerParam() => OK"

def demoAutoBalancerSetParam():
    print >> sys.stderr, "4. setAutoBalancerParam"
    abcp=hcf.abc_svc.getAutoBalancerParam()[1]
    abcp.default_zmp_offsets = [[0.1,0,0], [0.1,0,0], [0,0,0], [0,0,0]]
    hcf.abc_svc.setAutoBalancerParam(abcp)
    print >> sys.stderr, "  default_zmp_offsets setting check in start and stop"
    hcf.startAutoBalancer(["rleg", "lleg"]);
    hcf.stopAutoBalancer();
    ret=hcf.abc_svc.getAutoBalancerParam()
    flag = (ret[0] and numpy.allclose(ret[1].default_zmp_offsets, abcp.default_zmp_offsets, 1e-6))
    if flag:
        print >> sys.stderr, "  setAutoBalancerParam() => OK"
    assert (flag), (ret[0], ret[1].default_zmp_offsets, abcp.default_zmp_offsets)
    abcp.default_zmp_offsets = [[0,0,0], [0,0,0], [0,0,0], [0,0,0]]
    hcf.abc_svc.setAutoBalancerParam(abcp)

def demoAutoBalancerTestPoses():
    print >> sys.stderr, "5. change base height, base rot x, base rot y, and upper body while AutoBalancer mode"
    hcf.startAutoBalancer(["rleg", "lleg"]);
    testPoseList(pose_list, initial_pose)
    hcf.stopAutoBalancer();
    checkActualBaseAttitude()

def demoAutoBalancerStartStopCheck():
    print >> sys.stderr, "6. start stop check"
    abcp=hcf.abc_svc.getAutoBalancerParam()[1]
    abcp.default_zmp_offsets = [[-0.05,0.05,0], [-0.05,0.05,0], [0,0,0], [0,0,0]]
    hcf.abc_svc.setAutoBalancerParam(abcp)
    hcf.setMaxLogLength(1500)
    for pose in pose_list:
        hcf.seq_svc.setJointAngles(pose, 1.0)
        hcf.waitInterpolation()
        hcf.clearLog()
        hcf.startAutoBalancer(["rleg", "lleg"]);
        hcf.stopAutoBalancer();
        hcf.saveLog("/tmp/test-samplerobot-abc-startstop-{0}".format(pose_list.index(pose)))
    abcp.default_zmp_offsets = [[0,0,0], [0,0,0], [0,0,0], [0,0,0]]
    hcf.abc_svc.setAutoBalancerParam(abcp)
    hcf.seq_svc.setJointAngles(initial_pose, 1.0)
    hcf.waitInterpolation()
    checkActualBaseAttitude()

def demoAutoBalancerBalanceAgainstHandForce():
    print >> sys.stderr, "7. balance against hand force"
    hcf.startAutoBalancer(["rleg", "lleg"]);
    hcf.seq_svc.setWrenches([0,0,0,0,0,0,
                             0,0,0,0,0,0,
                             0,0,0,0,0,0,
                             0,0,-50,0,0,0,], 1.0); # rhsensor
    hcf.waitInterpolation();
    hcf.seq_svc.setWrenches([0,0,0,0,0,0,
                             0,0,0,0,0,0,
                             0,0,0,0,0,0,
                             0,0,0,0,0,0,], 1.0);
    hcf.waitInterpolation();
    hcf.stopAutoBalancer();
    checkActualBaseAttitude()

def demoAutoBalancerBalanceWithArms():
    print >> sys.stderr, "8. balance with arms"
    hcf.seq_svc.setJointAngles(four_legs_mode_pose, 1.0)
    hcf.waitInterpolation()
    abcp=hcf.abc_svc.getAutoBalancerParam()[1]
    abcp.leg_names = ['rleg', 'lleg', 'rarm', 'larm']
    hcf.abc_svc.setAutoBalancerParam(abcp)
    hcf.startAutoBalancer();
    print >> sys.stderr, "  startAutoBalancer with arms"
    hcf.stopAutoBalancer();
    print >> sys.stderr, "  stopAutoBalancer"
    abcp.leg_names = ['rleg', 'lleg']
    hcf.abc_svc.setAutoBalancerParam(abcp)
    checkActualBaseAttitude()
    hcf.seq_svc.setJointAngles(initial_pose, 1.0)
    hcf.waitInterpolation()

def demoGaitGeneratorGoPos():
    print >> sys.stderr, "1. goPos"
    hcf.startAutoBalancer();
    # initialize dst_foot_midcoords
    hcf.abc_svc.goPos(0,0,0)
    hcf.abc_svc.waitFootSteps()
    # gopos check 1
    goalx=0.1;goaly=0.1;goalth=20.0
    prev_dst_foot_midcoords=hcf.abc_svc.getFootstepParam()[1].dst_foot_midcoords
    hcf.abc_svc.goPos(goalx, goaly, goalth)
    hcf.abc_svc.waitFootSteps()
    checkGoPosParam(goalx, goaly, goalth, prev_dst_foot_midcoords)
    # gopos check 2
    goalx=-0.1;goaly=-0.1;goalth=-10.0
    prev_dst_foot_midcoords=hcf.abc_svc.getFootstepParam()[1].dst_foot_midcoords
    hcf.abc_svc.goPos(goalx, goaly, goalth)
    hcf.abc_svc.waitFootSteps()
    checkGoPosParam(goalx, goaly, goalth, prev_dst_foot_midcoords)
    checkActualBaseAttitude()
    print >> sys.stderr, "  goPos()=>OK"

def demoGaitGeneratorGoVelocity():
    print >> sys.stderr, "2. goVelocity and goStop"
    print >> sys.stderr, "  goVelocity few steps"
    hcf.abc_svc.goVelocity(-0.1, -0.05, -20)
    time.sleep(1)
    hcf.abc_svc.goStop()
    checkActualBaseAttitude()
    print >> sys.stderr, "  goVelocity few steps=>OK"
    print >> sys.stderr, "  Check discontinuity of COG by checking too large COG acc."
    hcf.setMaxLogLength(10000)
    hcf.clearLog()
    hcf.abc_svc.goVelocity(0,0,0) # One step overwrite
    hcf.abc_svc.goStop()
    hcf.saveLog("/tmp/test-abc-log");
    checkTooLargeABCCogAcc()

def demoGaitGeneratorSetFootSteps():
    print >> sys.stderr, "3. setFootSteps"
    hcf.setFootSteps([OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0,-0.09,0], [1,0,0,0], "rleg")]),
                      OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0,0.09,0], [1,0,0,0], "lleg")])])
    hcf.abc_svc.waitFootSteps()
    hcf.setFootSteps([OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0,-0.09,0], [1,0,0,0], "rleg")]),
                      OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0.15,0.09,0], [1,0,0,0], "lleg")]),
                      OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0.3,-0.09,0], [1,0,0,0], "rleg")]),
                      OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0.3,0.09,0], [1,0,0,0], "lleg")])])
    hcf.abc_svc.waitFootSteps()
    checkActualBaseAttitude()
    print >> sys.stderr, "  setFootSteps()=>OK"

def demoGaitGeneratorChangePoseWhileWalking():
    print >> sys.stderr, "4. Change base height, base rot x, base rot y, and upper body while walking"
    hcf.abc_svc.waitFootSteps()
    hcf.abc_svc.goVelocity(0,0,0)
    testPoseList(pose_list, initial_pose)
    hcf.abc_svc.goStop()
    checkActualBaseAttitude()

def demoGaitGeneratorGetParam():
    print >> sys.stderr, "5. getGaitGeneratorParam"
    ret = hcf.abc_svc.getGaitGeneratorParam()
    if ret[0]:
        print >> sys.stderr, "  getGaitGeneratorParam() => OK"

def demoGaitGeneratorSetParam():
    print >> sys.stderr, "6. setGaitGeneratorParam"
    ggp_org = hcf.abc_svc.getGaitGeneratorParam()[1]
    ggp = hcf.abc_svc.getGaitGeneratorParam()[1]
    ggp.default_step_time = 0.9
    ggp.default_step_height = 0.15
    ggp.default_double_support_ratio = 0.4
    ggp.swing_trajectory_delay_time_offset = 0.20
    ggp.default_orbit_type = OpenHRP.AutoBalancerService.RECTANGLE;
    hcf.abc_svc.setGaitGeneratorParam(ggp)
    ret = hcf.abc_svc.getGaitGeneratorParam()
    if ret[0] and ret[1].default_step_time == ggp.default_step_time and ret[1].default_step_height == ggp.default_step_height and ret[1].default_double_support_ratio == ggp.default_double_support_ratio and ret[1].default_orbit_type == ggp.default_orbit_type:
        print >> sys.stderr, "  setGaitGeneratorParam() => OK"
    hcf.abc_svc.goPos(0.2,0,0)
    hcf.abc_svc.waitFootSteps()
    hcf.abc_svc.setGaitGeneratorParam(ggp_org) # revert parameter

def demoGaitGeneratorNonDefaultStrideStop():
    print >> sys.stderr, "7. non-default stride"
    hcf.setFootSteps([OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0,-0.09,0], [1,0,0,0], "rleg")]),
                      OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0.15,0.09,0], [1,0,0,0], "lleg")])])
    hcf.abc_svc.waitFootSteps()
    hcf.setFootSteps([OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0,-0.09,0], [1,0,0,0], "rleg")]),
                      OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0,0.09,0], [1,0,0,0], "lleg")])])
    hcf.abc_svc.waitFootSteps()
    checkActualBaseAttitude()
    print >> sys.stderr, "  Non default Stride()=>OK"

def demoGaitGeneratorToeHeelContact():
    print >> sys.stderr, "8. Use toe heel contact"
    ggp=hcf.abc_svc.getGaitGeneratorParam()[1];
    ggp.toe_pos_offset_x = 1e-3*182.0;
    ggp.heel_pos_offset_x = 1e-3*-72.0;
    ggp.toe_zmp_offset_x = 1e-3*182.0;
    ggp.heel_zmp_offset_x = 1e-3*-72.0;
    ggp.toe_angle = 20;
    ggp.heel_angle = 10;
    hcf.abc_svc.setGaitGeneratorParam(ggp);
    hcf.abc_svc.goPos(0.3, 0, 0);
    hcf.abc_svc.waitFootSteps()
    ggp.toe_angle = 0;
    ggp.heel_angle = 0;
    hcf.abc_svc.setGaitGeneratorParam(ggp);
    checkActualBaseAttitude()
    print >> sys.stderr, "  Toe heel contact=>OK"

def demoGaitGeneratorStopStartSyncCheck():
    print >> sys.stderr, "9. Stop and start auto balancer sync check2"
    print >> sys.stderr, "  Check 9-1 Sync after setFootSteps"
    hcf.startAutoBalancer();
    hcf.setFootSteps([OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0,-0.09,0], [1,0,0,0], "rleg")]),
                      OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0.1,0.09,0], [1,0,0,0], "lleg")])]);
    hcf.abc_svc.waitFootSteps();
    hcf.stopAutoBalancer();
    print >> sys.stderr, "    Sync after setFootSteps => OK"
    print >> sys.stderr, "  Check 9-2 Sync from setJointAngles at the beginning"
    open_stride_pose = [0.00026722677758058496, -0.3170503560247552, -0.0002054613599000865, 0.8240549352035262, -0.5061434785447525, -8.67443660992421e-05, 0.3112899999999996, -0.15948099999999998, -0.11539900000000003, -0.6362769999999993, 0.0, 0.0, 0.0, 0.00023087433689200683, -0.4751295978345554, -0.00021953834197007937, 0.8048588066686679, -0.3288687069275527, -8.676469399681631e-05, 0.3112899999999996, 0.15948099999999998, 0.11539900000000003, -0.6362769999999993, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    hcf.seq_svc.setJointAngles(open_stride_pose, 2.0);
    hcf.waitInterpolation();
    hcf.startAutoBalancer();
    hcf.setFootSteps([OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0,-0.09,0], [1,0,0,0], "rleg")]),
                      OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0.1,0.09,0], [1,0,0,0], "lleg")])]);
    hcf.abc_svc.waitFootSteps();
    hcf.stopAutoBalancer();
    print >> sys.stderr, "    Sync from setJointAngle at the beginning => OK"
    print >> sys.stderr, "  Check 9-3 Sync from setJointAngles"
    hcf.startAutoBalancer();
    hcf.seq_svc.setJointAngles(initial_pose, 2.0);
    hcf.waitInterpolation();
    hcf.stopAutoBalancer();
    print >> sys.stderr, "    Sync from setJointAngle => OK"

def demoGaitGeneratorEmergencyStop():
    print >> sys.stderr, "10. Emergency stop"
    hcf.startAutoBalancer();
    hcf.abc_svc.goPos(0,0,90);
    print >> sys.stderr, "  Start goPos and wait for 4 steps"
    for idx in range(4): # Wait for 4 steps including initial double support phase
        # Wait for 1 steps
        hcf.seq_svc.setJointAngles(initial_pose, hcf.abc_svc.getGaitGeneratorParam()[1].default_step_time);
        hcf.waitInterpolation();
    print >> sys.stderr, "  Emergency stoping"
    hcf.abc_svc.emergencyStop();
    print >> sys.stderr, "  Align foot steps"
    hcf.abc_svc.goPos(0,0,0);
    checkActualBaseAttitude()

def demoGaitGeneratorGetRemainingSteps():
    print >> sys.stderr, "11. Get remaining foot steps"
    hcf.abc_svc.goPos(0.3,0.1,15);
    fslist=hcf.abc_svc.getRemainingFootstepSequence()[1]
    while fslist != []:
        fslist=hcf.abc_svc.getRemainingFootstepSequence()[1]
        print >> sys.stderr, "  Remaining footstep ", len(fslist)
        # Wait for 1 step
        hcf.seq_svc.setJointAngles(initial_pose, hcf.abc_svc.getGaitGeneratorParam()[1].default_step_time);
        hcf.waitInterpolation();
    checkActualBaseAttitude()

def demoGaitGeneratorChangeStepParam():
    print >> sys.stderr, "12. Change step param with setFootSteps"
    ggp_org=hcf.abc_svc.getGaitGeneratorParam()[1];
    # dummy setting
    ggp=hcf.abc_svc.getGaitGeneratorParam()[1];
    ggp.toe_angle = 50;
    ggp.heel_angle = 50;
    hcf.abc_svc.setGaitGeneratorParam(ggp);
    hcf.setFootStepsWithParam([OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0,-0.09,0], [1,0,0,0], "rleg")]),
                               OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0.1,0.09,0], [1,0,0,0], "lleg")]),
                               OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0.2,-0.09,0], [1,0,0,0], "rleg")]),
                               OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0.2,0.09,0], [1,0,0,0], "lleg")])],
                              [OpenHRP.AutoBalancerService.StepParams([OpenHRP.AutoBalancerService.StepParam(step_height=0.0, step_time=1.0, toe_angle=0.0, heel_angle=0.0)]),
                               OpenHRP.AutoBalancerService.StepParams([OpenHRP.AutoBalancerService.StepParam(step_height=0.05, step_time=2.0, toe_angle=0.0, heel_angle=0.0)]),
                               OpenHRP.AutoBalancerService.StepParams([OpenHRP.AutoBalancerService.StepParam(step_height=0.05, step_time=1.0, toe_angle=0.0, heel_angle=0.0)]),
                               OpenHRP.AutoBalancerService.StepParams([OpenHRP.AutoBalancerService.StepParam(step_height=0.05, step_time=2.0, toe_angle=0.0, heel_angle=0.0)])])
    hcf.abc_svc.waitFootSteps()
    hcf.setFootStepsWithParam([OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0,-0.09,0], [1,0,0,0], "rleg")]),
                               OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0.1,0.09,0], [1,0,0,0], "lleg")]),
                               OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0.2,-0.09,0], [1,0,0,0], "rleg")]),
                               OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0.2,0.09,0], [1,0,0,0], "lleg")])],
                              [OpenHRP.AutoBalancerService.StepParams([OpenHRP.AutoBalancerService.StepParam(step_height=0.0, step_time=1.0, toe_angle=0.0, heel_angle=0.0)]),
                               OpenHRP.AutoBalancerService.StepParams([OpenHRP.AutoBalancerService.StepParam(step_height=0.1, step_time=1.0, toe_angle=0.0, heel_angle=0.0)]),
                               OpenHRP.AutoBalancerService.StepParams([OpenHRP.AutoBalancerService.StepParam(step_height=0.05, step_time=1.0, toe_angle=0.0, heel_angle=0.0)]),
                               OpenHRP.AutoBalancerService.StepParams([OpenHRP.AutoBalancerService.StepParam(step_height=0.1, step_time=1.0, toe_angle=0.0, heel_angle=0.0)])])
    hcf.abc_svc.waitFootSteps()
    hcf.setFootStepsWithParam([OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0,-0.09,0], [1,0,0,0], "rleg")]),
                               OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0.1,0.09,0], [1,0,0,0], "lleg")]),
                               OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0.2,-0.09,0], [1,0,0,0], "rleg")]),
                               OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0.2,0.09,0], [1,0,0,0], "lleg")])],
                              [OpenHRP.AutoBalancerService.StepParams([OpenHRP.AutoBalancerService.StepParam(step_height=0.0, step_time=1.0, toe_angle=0.0, heel_angle=0.0)]),
                               OpenHRP.AutoBalancerService.StepParams([OpenHRP.AutoBalancerService.StepParam(step_height=0.05, step_time=1.0, toe_angle=0.0, heel_angle=0.0)]),
                               OpenHRP.AutoBalancerService.StepParams([OpenHRP.AutoBalancerService.StepParam(step_height=0.05, step_time=1.0, toe_angle=20.0, heel_angle=5.0)]),
                               OpenHRP.AutoBalancerService.StepParams([OpenHRP.AutoBalancerService.StepParam(step_height=0.05, step_time=1.0, toe_angle=10.0, heel_angle=10.0)])])
    hcf.abc_svc.waitFootSteps()
    hcf.abc_svc.setGaitGeneratorParam(ggp_org);
    checkActualBaseAttitude()

def demoGaitGeneratorOverwriteFootsteps(overwrite_offset_idx = 1):
    print >> sys.stderr, "13. Overwrite footsteps during walking."
    hcf.startAutoBalancer()
    demoGaitGeneratorOverwriteFootstepsBase("x", overwrite_offset_idx, True) # Overwrite by X direction foot steps
    hcf.seq_svc.setJointAngles(initial_pose, 1.0*overwrite_offset_idx)
    hcf.seq_svc.waitInterpolation()
    demoGaitGeneratorOverwriteFootstepsBase("y", overwrite_offset_idx, True) # Overwrite by Y direction foot steps
    hcf.seq_svc.setJointAngles(initial_pose, 1.0*overwrite_offset_idx)
    hcf.seq_svc.waitInterpolation()
    demoGaitGeneratorOverwriteFootstepsBase("x", overwrite_offset_idx, True) # Overwrite by X direction foot steps
    hcf.abc_svc.waitFootSteps()
    checkActualBaseAttitude()

def demoGaitGeneratorOverwriteFootstepsBase(axis, overwrite_offset_idx = 1, init_fs = False):
    if init_fs:
        hcf.setFootSteps([OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0,  -0.09,0], [1,0,0,0], "rleg")]),
                          OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0.1, 0.09,0], [1,0,0,0], "lleg")]),
                          OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0.2,-0.09,0], [1,0,0,0], "rleg")]),
                          OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0.3, 0.09,0], [1,0,0,0], "lleg")]),
                          OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0.4,-0.09,0], [1,0,0,0], "rleg")]),
                          OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0.4, 0.09,0], [1,0,0,0], "lleg")])]);
    print >> sys.stderr, "  Overwrite footsteps ", overwrite_offset_idx
    # Get remaining footstep
    [remain_fs, current_fs_idx]=hcf.abc_svc.getRemainingFootstepSequence()[1:]
    #print >> sys.stderr, remain_fs
    print >> sys.stderr, "    Remaining legs = ", map(lambda fs : fs.leg, remain_fs)
    print >> sys.stderr, "    Remaining idx  = ", map(lambda idx : current_fs_idx+idx, range(len(remain_fs)))
    # Footstep index to be overwritten
    overwrite_fs_idx = current_fs_idx + overwrite_offset_idx
    print >> sys.stderr, "    Overwrite index = ",overwrite_fs_idx, ", leg = ", remain_fs[overwrite_offset_idx].leg
    # Calc new footsteps
    import numpy
    support_fs = remain_fs[overwrite_offset_idx-1] # support fs before overwritten fs
    if axis == "x":
        pos_offset = [0.1, 0, 0]
        pos_offset2 = [0.2, 0, 0]
    else:
        pos_offset = [0, (0.1 if support_fs.leg =='rleg' else -0.1), 0]
        pos_offset2 = pos_offset
    fpos1=list(numpy.array(support_fs.pos) + numpy.array([0, 2.0*(0.09 if support_fs.leg =='rleg' else -0.09) ,0]) + numpy.array(pos_offset))
    fpos2=list(numpy.array(support_fs.pos) + numpy.array(pos_offset))
    fpos3=list(numpy.array(support_fs.pos) + numpy.array([0, 2.0*(0.09 if support_fs.leg =='rleg' else -0.09) ,0]) + numpy.array(pos_offset2))
    fpos4=list(numpy.array(support_fs.pos) + numpy.array(pos_offset2))
    new_fs =[OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep(support_fs.pos, [1,0,0,0], support_fs.leg)]),
             OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep(fpos1,          [1,0,0,0], "lleg" if support_fs.leg =='rleg' else "rleg")]),
             OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep(fpos2,          [1,0,0,0], support_fs.leg)]),
             OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep(fpos3,          [1,0,0,0], "lleg" if support_fs.leg =='rleg' else "rleg")]),
             OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep(fpos4,          [1,0,0,0], support_fs.leg)])]
    hcf.abc_svc.setFootSteps(new_fs, overwrite_fs_idx);

def demoGaitGeneratorFixHand():
    print >> sys.stderr, "14. Fix arm walking"
    hcf.stopAutoBalancer()
    hcf.startAutoBalancer()
    # Set pose
    abcp=hcf.abc_svc.getAutoBalancerParam()[1]
    abcp.default_zmp_offsets=[[0.01, 0.0, 0.0], [0.01, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]] # Setting default_zmp_offsets is not necessary for fix mode. Just for debugging for default_zmp_offsets in hand fix mode.
    hcf.abc_svc.setAutoBalancerParam(abcp)
    dualarm_push_pose=[-3.998549e-05,-0.710564,-0.000264,1.41027,-0.680767,-2.335251e-05,-0.541944,-0.091558,0.122667,-1.02616,-1.71287,-0.056837,1.5708,-3.996804e-05,-0.710511,-0.000264,1.41016,-0.680706,-2.333505e-05,-0.542,0.091393,-0.122578,-1.02608,1.71267,-0.05677,-1.5708,0.006809,0.000101,-0.000163]
    hcf.seq_svc.setJointAngles(dualarm_push_pose, 1.0)
    hcf.waitInterpolation()
    print >> sys.stderr, "  Walk without fixing arm" 
    abcp=hcf.abc_svc.getAutoBalancerParam()[1]
    abcp.is_hand_fix_mode=False
    hcf.abc_svc.setAutoBalancerParam(abcp)
    hcf.abc_svc.goPos(0.3,0,0)
    hcf.abc_svc.waitFootSteps()
    hcf.abc_svc.goPos(0,0.2,0)
    hcf.abc_svc.waitFootSteps()
    hcf.abc_svc.goPos(0,0,30)
    hcf.abc_svc.waitFootSteps()
    print >> sys.stderr, "  Walk with fixing arm" 
    abcp=hcf.abc_svc.getAutoBalancerParam()[1]
    abcp.is_hand_fix_mode=True
    hcf.abc_svc.setAutoBalancerParam(abcp)
    hcf.abc_svc.goPos(0.3,0,0)
    hcf.abc_svc.waitFootSteps()
    hcf.abc_svc.goPos(0,-0.2,0)
    hcf.abc_svc.waitFootSteps()
    hcf.abc_svc.goPos(0,0,-30)
    hcf.abc_svc.waitFootSteps()
    abcp=hcf.abc_svc.getAutoBalancerParam()[1]
    abcp.is_hand_fix_mode=False
    abcp.default_zmp_offsets=[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    hcf.abc_svc.setAutoBalancerParam(abcp)
    ref_rpy = checkParameterFromLog("baseRpyOut", rtc_name="abc")
    hcf.stopAutoBalancer()
    checkActualBaseAttitude(ref_rpy)
    print >> sys.stderr, "  Fix hand=>OK"

def demoGaitGeneratorOverwriteCurrentFootstep():
    print >> sys.stderr, "15. Overwrite current footstep"
    hcf.seq_svc.setJointAngles(initial_pose, 1.0)
    hcf.waitInterpolation()
    hcf.startAutoBalancer()
    # decrease zmp weight for arms
    orig_ggp = hcf.abc_svc.getGaitGeneratorParam()[1]
    ggp = hcf.abc_svc.getGaitGeneratorParam()[1]
    ggp.overwritable_footstep_index_offset = 0
    ggp.default_orbit_type=OpenHRP.AutoBalancerService.RECTANGLE
    hcf.abc_svc.setGaitGeneratorParam(ggp)
    # start walking
    hcf.abc_svc.goVelocity(0,0,0);
    hcf.seq_svc.setJointAngles(initial_pose, 2.0);hcf.waitInterpolation() #  wait 2 step using dummy waitInterpolation
    hcf.abc_svc.goVelocity(0.1,0,0);
    hcf.seq_svc.setJointAngles(initial_pose, 2.0);hcf.waitInterpolation() #  wait 2 step using dummy waitInterpolation
    hcf.abc_svc.goVelocity(0,0.1,0);
    hcf.seq_svc.setJointAngles(initial_pose, 2.0);hcf.waitInterpolation() #  wait 2 step using dummy waitInterpolation
    hcf.abc_svc.goStop()
    checkActualBaseAttitude()
    print >> sys.stderr, "  Overwrite current footstep=>OK"
    # reset params
    hcf.abc_svc.setGaitGeneratorParam(orig_ggp)

def demoGaitGeneratorGoPosOverwrite():
    print >> sys.stderr, "16. goPos overwriting"
    hcf.startAutoBalancer();
    print >> sys.stderr, "  Overwrite goPos by goPos"
    goalx=0.3;goaly=0.1;goalth=15.0
    prev_dst_foot_midcoords=hcf.abc_svc.getFootstepParam()[1].dst_foot_midcoords
    hcf.abc_svc.goPos(0.2,-0.1,-5) # initial gopos
    hcf.seq_svc.setJointAngles(initial_pose, 2.0);hcf.waitInterpolation() #  wait 2 step using dummy waitInterpolation
    hcf.abc_svc.goPos(goalx,goaly,goalth) # overwrite gopos
    hcf.abc_svc.waitFootSteps()
    checkGoPosParam(goalx, goaly, goalth, prev_dst_foot_midcoords)
    print >> sys.stderr, "  Overwrite setFootSteps by goPos"
    prev_dst_foot_midcoords=hcf.abc_svc.getFootstepParam()[1].dst_foot_midcoords
    hcf.setFootSteps([OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0,-0.09,0], [1,0,0,0], "rleg")]),
                      OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0.1,0.09,0], [1,0,0,0], "lleg")]),
                      OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0.2,-0.09,0], [1,0,0,0], "rleg")]),
                      OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0.3,0.09,0], [1,0,0,0], "lleg")]),
                      OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0.3,-0.09,0], [1,0,0,0], "rleg")])
                      ]) # initial setfootsteps
    hcf.seq_svc.setJointAngles(initial_pose, 2.0);hcf.waitInterpolation() #  wait 2 step using dummy waitInterpolation
    hcf.abc_svc.goPos(goalx,goaly,goalth) # overwrite gopos
    hcf.abc_svc.waitFootSteps()
    checkGoPosParam(goalx, goaly, goalth, prev_dst_foot_midcoords)

def demoGaitGeneratorGrasplessManipMode():
    print >> sys.stderr, "17. Graspless manip mode"
    hcf.startAutoBalancer();
    # Initialize and pose define
    dualarm_push_pose=[-3.998549e-05,-0.710564,-0.000264,1.41027,-0.680767,-2.335251e-05,-0.541944,-0.091558,0.122667,-1.02616,-1.71287,-0.056837,1.5708,-3.996804e-05,-0.710511,-0.000264,1.41016,-0.680706,-2.333505e-05,-0.542,0.091393,-0.122578,-1.02608,1.71267,-0.05677,-1.5708,0.006809,0.000101,-0.000163]
    hcf.seq_svc.setJointAngles(dualarm_push_pose, 0.5)
    hcf.waitInterpolation()
    # hands 50[mm] fwd from dualarm_push_pose
    av_fwd = [-5.579249e-05,-0.760285,-0.000277,1.44619,-0.660772,-2.615057e-05,-0.7752,-0.080815,0.116555,-0.935667,-1.70514,-0.045373,1.309,-5.577374e-05,-0.760232,-0.000277,1.44608,-0.660715,-2.613350e-05,-0.77525,0.080663,-0.116463,-0.935597,1.70494,-0.045325,-1.309,0.157668,0.000123,-0.000152]
    # hands 50[mm] bwd from dualarm_push_pose
    av_bwd = [-1.901820e-05,-0.641174,-0.00025,1.36927,-0.717047,-2.260319e-05,-0.305537,-0.099557,0.134675,-1.04208,-1.72497,-0.065256,1.309,-1.900236e-05,-0.641122,-0.00025,1.36915,-0.71698,-2.258509e-05,-0.305624,0.099383,-0.134605,-1.04197,1.72476,-0.06517,-1.309,-0.22394,5.625198e-05,-0.000165]
    # hands 50[mm] right from dualarm_push_pose
    av_right = [-0.005678,-0.711398,0.006148,1.40852,-0.678974,0.011103,-0.575284,-0.179786,0.092155,-0.999366,-1.74805,0.048205,1.309,-0.005686,-0.710309,0.006143,1.4098,-0.681345,0.011103,-0.520691,0.002033,-0.154878,-1.05585,1.6731,-0.161177,-1.309,0.015053,0.024788,-0.023196]
    # hands 50[mm] left from dualarm_push_pose
    av_left = [0.005607,-0.71036,-0.006671,1.40991,-0.681404,-0.011151,-0.52064,-0.002193,0.154967,-1.05593,-1.67329,-0.161246,1.309,0.005598,-0.711343,-0.006677,1.4084,-0.67891,-0.011151,-0.575342,0.179622,-0.092066,-0.999287,1.74785,0.04827,-1.309,0.015056,-0.024583,0.02287]
    # hands 10[deg] right turn from dualarm_push_pose
    av_rturn = [0.023512,-0.71245,0.014216,1.40899,-0.677868,-0.01575,-0.462682,0.040789,0.154221,-1.10667,-1.66067,-0.2349,1.309,0.023442,-0.708029,0.014161,1.40823,-0.68153,-0.015763,-0.61987,0.217174,-0.105089,-0.949927,1.75163,0.120793,-1.309,0.013747,-0.058774,-0.084435]
    # hands 10[deg] left turn from dualarm_push_pose
    av_lturn = [-0.023522,-0.708079,-0.014689,1.40835,-0.681597,0.015717,-0.619803,-0.217337,0.105179,-0.950004,-1.75184,0.120733,1.309,-0.02359,-0.712393,-0.014744,1.40889,-0.677813,0.0157,-0.462722,-0.040951,-0.154135,-1.10659,1.66048,-0.234826,-1.309,0.013715,0.058979,0.084109]
    # parameter setting
    org_abcp=hcf.abc_svc.getAutoBalancerParam()[1]
    abcp=hcf.abc_svc.getAutoBalancerParam()[1]
    abcp.graspless_manip_mode=True
    abcp.is_hand_fix_mode=True
    abcp.graspless_manip_reference_trans_rot=[1.0, 0.0, 0.0, 1.365307e-06] # trans rot for dualarm_push_pose
    abcp.graspless_manip_reference_trans_pos=[0.450037, 1.049436e-06, 0.869818] # trans pos for dualarm_push_pose
    abcp.graspless_manip_p_gain=[1,1,1]
    hcf.abc_svc.setAutoBalancerParam(abcp)
    hcf.co_svc.disableCollisionDetection()
    # Check one foot_midcoords movement
    gv_pose_list = [av_fwd, av_bwd, av_left, av_right, av_lturn, av_rturn]
    ref_footmid_diff = [[50*1e-3,0,0],
                        [-50*1e-3,0,0],
                        [0, 0.5*50*1e-3,0], # 0.5->inside limitation
                        [0,-0.5*50*1e-3,0], # 0.5->inside limitation
                        [0,0, 0.5*10], # 0.5->inside limitation
                        [0,0,-0.5*10]] # 0.5->inside limitation
    ret=True
    hcf.abc_svc.waitFootSteps()
    for idx in range(len(gv_pose_list)):
        pose = gv_pose_list[idx]
        prev_dst_foot_midcoords=hcf.abc_svc.getFootstepParam()[1].dst_foot_midcoords
        hcf.abc_svc.goVelocity(0,0,0);
        hcf.seq_svc.setJointAngles(pose, 0.4)
        hcf.waitInterpolation()
        hcf.seq_svc.setJointAngles(pose, 1.6);hcf.waitInterpolation() # Dummy 2step
        hcf.abc_svc.goStop()
        diff=numpy.array(ref_footmid_diff[idx])-numpy.array(calcDiffFootMidCoords(prev_dst_foot_midcoords))
        if idx == 4 or idx == 5:
            tmpret = abs(diff[2]) < 1.0 # TODO, check pos
        else:
            tmpret = abs(diff[0]) < 1e-3 and abs(diff[1]) < 1e-3 and abs(diff[2]) < 1.0
        ret = ret and tmpret
        print >> sys.stderr, "  ret = ", tmpret, ", diff = ", diff
    # Finishing
    if ret:
        print >> sys.stderr, "  total is OK"
    assert(ret)
    hcf.co_svc.enableCollisionDetection()
    hcf.seq_svc.setJointAngles(dualarm_push_pose, 0.5)
    hcf.waitInterpolation()
    hcf.abc_svc.setAutoBalancerParam(org_abcp)

def demoGaitGeneratorSetFootStepsWithArms():
    print >> sys.stderr, "18. Trot Walking"
    hcf.stopAutoBalancer()
    hcf.seq_svc.setJointAngles(four_legs_mode_pose, 1.0)
    hcf.waitInterpolation()
    # use arms
    orig_abcp = hcf.abc_svc.getAutoBalancerParam()[1]
    abcp = hcf.abc_svc.getAutoBalancerParam()[1]
    abcp.leg_names = ['rleg', 'lleg', 'rarm', 'larm']
    hcf.abc_svc.setAutoBalancerParam(abcp)
    # decrease zmp weight for arms
    orig_ggp = hcf.abc_svc.getGaitGeneratorParam()[1]
    ggp = hcf.abc_svc.getGaitGeneratorParam()[1]
    ggp.zmp_weight_map = [1.0, 1.0, 0.01, 0.01]
    ggp.default_step_height = 0.01
    hcf.abc_svc.setGaitGeneratorParam(ggp)
    # start walking
    hcf.startAutoBalancer()
    hcf.setFootSteps([OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0,-0.09,0], [1,0,0,0], "rleg"),
                                                             OpenHRP.AutoBalancerService.Footstep([0.23,0.21,0.86], [1,0,0,0], "larm")]),
                      OpenHRP.AutoBalancerService.Footsteps([OpenHRP.AutoBalancerService.Footstep([0,0.09,0], [1,0,0,0], "lleg"),
                                                             OpenHRP.AutoBalancerService.Footstep([0.23,-0.21,0.86], [1,0,0,0], "rarm")])])
    hcf.abc_svc.waitFootSteps()
    checkActualBaseAttitude()
    print >> sys.stderr, "  setFootSteps()=>OK"
    # reset params
    hcf.stopAutoBalancer()
    hcf.abc_svc.setAutoBalancerParam(orig_abcp)
    hcf.abc_svc.setGaitGeneratorParam(orig_ggp)
    hcf.startAutoBalancer()

def demoGaitGeneratorChangeStrideLimitationType():
    print >> sys.stderr, "19. Change stride limitation type to CIRCLE"
    hcf.startAutoBalancer();
    # initialize dst_foot_midcoords
    hcf.abc_svc.goPos(0,0,0)
    hcf.abc_svc.waitFootSteps()
    # set params
    orig_ggp = hcf.abc_svc.getGaitGeneratorParam()[1]
    ggp = hcf.abc_svc.getGaitGeneratorParam()[1]
    ggp.stride_limitation_type = OpenHRP.AutoBalancerService.CIRCLE
    ggp.overwritable_stride_limitation = [0.15, 0.1, 0.25, 0.1]
    ggp.leg_margin = [182.0*1e-3, 72.0*1e-3, 71.12*1e-3, 71.12*1e-3]
    hcf.abc_svc.setGaitGeneratorParam(ggp)
    # gopos check 1
    goalx=0.3;goaly=0.3;goalth=20.0
    prev_dst_foot_midcoords=hcf.abc_svc.getFootstepParam()[1].dst_foot_midcoords
    hcf.abc_svc.goPos(goalx, goaly, goalth)
    hcf.abc_svc.waitFootSteps()
    checkGoPosParam(goalx, goaly, goalth, prev_dst_foot_midcoords)
    # gopos check 2
    goalx=-0.3;goaly=-0.3;goalth=-10.0
    prev_dst_foot_midcoords=hcf.abc_svc.getFootstepParam()[1].dst_foot_midcoords
    hcf.abc_svc.goPos(goalx, goaly, goalth)
    hcf.abc_svc.waitFootSteps()
    checkGoPosParam(goalx, goaly, goalth, prev_dst_foot_midcoords)
    checkActualBaseAttitude()
    print >> sys.stderr, "  Change stride limitation type to CIRCLE=>OK"
    # reset params
    hcf.abc_svc.setGaitGeneratorParam(orig_ggp)

def demoStandingPosResetting():
    print >> sys.stderr, "demoStandingPosResetting"
    hcf.abc_svc.goPos(0,0,math.degrees(-1*checkParameterFromLog("WAIST")[5])); # Rot yaw
    hcf.abc_svc.waitFootSteps()
    hcf.abc_svc.goPos(0,-1*checkParameterFromLog("WAIST")[1],0); # Pos Y
    hcf.abc_svc.waitFootSteps()

def demo():
    init()
    if hrpsys_version >= '315.5.0':
        # sample for AutoBalancer mode
        demoAutoBalancerFixFeet()
        demoAutoBalancerFixFeetHands()
        demoAutoBalancerGetParam()
        demoAutoBalancerSetParam()
        demoAutoBalancerTestPoses()
        demoAutoBalancerStartStopCheck()
        demoAutoBalancerBalanceAgainstHandForce()
        demoAutoBalancerBalanceWithArms()
        # sample for walk pattern generation by AutoBalancer RTC
        demoGaitGeneratorGoPos()
        demoGaitGeneratorGoVelocity()
        demoGaitGeneratorSetFootSteps()
        demoGaitGeneratorChangePoseWhileWalking()
        demoGaitGeneratorGetParam()
        demoGaitGeneratorSetParam()
        demoGaitGeneratorNonDefaultStrideStop()
        demoGaitGeneratorToeHeelContact()
        demoGaitGeneratorStopStartSyncCheck()
        demoGaitGeneratorEmergencyStop()
        demoGaitGeneratorGetRemainingSteps()
        demoGaitGeneratorChangeStepParam()
        demoGaitGeneratorOverwriteFootsteps()
        demoGaitGeneratorOverwriteFootsteps(2)
        demoStandingPosResetting()
        demoGaitGeneratorFixHand()
        demoGaitGeneratorOverwriteCurrentFootstep()
        demoGaitGeneratorGoPosOverwrite()
        demoGaitGeneratorGrasplessManipMode()
        demoGaitGeneratorSetFootStepsWithArms()
        demoGaitGeneratorChangeStrideLimitationType()

if __name__ == '__main__':
    demo()


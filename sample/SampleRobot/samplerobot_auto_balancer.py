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
    global hcf, initial_pose, arm_front_pose, half_sitting_pose, root_rot_x_pose, root_rot_y_pose, pose_list, hrpsys_version, four_legs_mode_pose
    hcf = HrpsysConfigurator()
    hcf.getRTCList = hcf.getRTCListUnstable
    hcf.init ("SampleRobot(Robot)0", "$(PROJECT_DIR)/../model/sample1.wrl")
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

def checkActualBaseAttitude():
    rpy = rtm.readDataPort(hcf.rh.port("WAIST")).data.orientation
    ret = math.degrees(rpy.r) < 0.1 and math.degrees(rpy.p) < 0.1
    print >> sys.stderr, "  actual base rpy = ", ret, "(", rpy, ")"
    assert (ret)
    return ret

def demoAutoBalancerFixFeet ():
    print >> sys.stderr, "1. AutoBalancer mode by fixing feet"
    hcf.startAutoBalancer();
    hcf.seq_svc.setJointAngles(arm_front_pose, 1.0)
    hcf.waitInterpolation()
    hcf.seq_svc.setJointAngles(initial_pose, 1.0)
    hcf.waitInterpolation()
    hcf.stopAutoBalancer();
    checkActualBaseAttitude()
    print >> sys.stderr, "  Start and Stop AutoBalancer by fixing feet=>OK"

def demoAutoBalancerFixFeetHands ():
    print >> sys.stderr, "2. AutoBalancer mode by fixing hands and feet"
    hcf.startAutoBalancer(["rleg", "lleg", "rarm", "larm"])
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
    hcf.startAutoBalancer();
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
    hcf.startAutoBalancer();
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
        hcf.startAutoBalancer();
        hcf.stopAutoBalancer();
        hcf.saveLog("/tmp/test-samplerobot-abc-startstop-{0}".format(pose_list.index(pose)))
    abcp.default_zmp_offsets = [[0,0,0], [0,0,0], [0,0,0], [0,0,0]]
    hcf.abc_svc.setAutoBalancerParam(abcp)
    hcf.seq_svc.setJointAngles(initial_pose, 1.0)
    hcf.waitInterpolation()
    checkActualBaseAttitude()

def demoAutoBalancerBalanceAgainstHandForce():
    print >> sys.stderr, "7. balance against hand force"
    hcf.startAutoBalancer();
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
    hcf.abc_svc.goPos(0.1, 0.05, 20)
    hcf.abc_svc.waitFootSteps()
    checkActualBaseAttitude()
    print >> sys.stderr, "  goPos()=>OK"

def demoGaitGeneratorGoVelocity():
    print >> sys.stderr, "2. goVelocity and goStop"
    hcf.abc_svc.goVelocity(-0.1, -0.05, -20)
    time.sleep(1)
    hcf.abc_svc.goStop()
    checkActualBaseAttitude()
    print >> sys.stderr, "  goVelocity()=>OK"

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
    hcf.startAutoBalancer(['rleg', 'lleg', 'rarm', 'larm'])
    # Set pose
    abcp=hcf.abc_svc.getAutoBalancerParam()[1]
    abcp.default_zmp_offsets=[[0.01, 0.0, 0.0], [0.01, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]] # Setting default_zmp_offsets is not necessary for fix mode. Just for debugging for default_zmp_offsets in hand fix mode.
    hcf.abc_svc.setAutoBalancerParam(abcp)
    dualarm_push_pose=[-3.998549e-05,-0.710564,-0.000264,1.41027,-0.680767,-2.335251e-05,-0.541944,-0.091558,0.122667,-1.02616,-1.71287,-0.056837,1.5708,-3.996804e-05,-0.710511,-0.000264,1.41016,-0.680706,-2.333505e-05,-0.542,0.091393,-0.122578,-1.02608,1.71267,-0.05677,-1.5708,0.006809,0.000101,-0.000163]
    hcf.seq_svc.setJointAngles(dualarm_push_pose, 2.0)
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
    hcf.abc_svc.goPos(0,0.2,0)
    hcf.abc_svc.waitFootSteps()
    hcf.abc_svc.goPos(0,0,30)
    hcf.abc_svc.waitFootSteps()
    abcp=hcf.abc_svc.getAutoBalancerParam()[1]
    abcp.default_zmp_offsets=[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    hcf.abc_svc.setAutoBalancerParam(abcp)
    hcf.stopAutoBalancer()
    checkActualBaseAttitude()
    print >> sys.stderr, "  Fix hand=>OK"

def demoGaitGeneratorSetFootStepsWithArms():
    print >> sys.stderr, "15. Trot Walking"
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
        demoGaitGeneratorFixHand()
        demoGaitGeneratorSetFootStepsWithArms()

if __name__ == '__main__':
    demo()


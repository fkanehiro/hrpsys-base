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

from subprocess import check_output

# Tempolarily remove tc which is limit position range
def getRTCList ():
    return [x for x in hcf.getRTCListUnstable() if x[0]!='tc']

def init ():
    global hcf, initial_pose, limit_table_list, bodyinfo, hrpsys_version
    hcf = HrpsysConfigurator()
    hcf.getRTCList = getRTCList
    hcf.init ("SampleRobot(Robot)0", "$(PROJECT_DIR)/../model/sample1.wrl")
    initial_pose = [-7.779e-005,  -0.378613,  -0.000209793,  0.832038,  -0.452564,  0.000244781,  0.31129,  -0.159481,  -0.115399,  -0.636277,  0,  0,  0,  -7.77902e-005,  -0.378613,  -0.000209794,  0.832038,  -0.452564,  0.000244781,  0.31129,  0.159481,  0.115399,  -0.636277,  0,  0,  0,  0,  0,  0]
    # load joint limit table from conf file
    HRPSYS_DIR=check_output(['pkg-config', 'hrpsys-base', '--variable=prefix']).rstrip()
    f=open("{}/share/hrpsys/samples/SampleRobot/SampleRobot.500.el.conf".format(HRPSYS_DIR))
    limit_table_str=[x for x in f.readlines() if x.find("joint_limit_table") > -1][0]
    limit_table_list=limit_table_str.split(":")[1:]
    f.close()
    # set bodyinfo
    bodyinfo=hcf.getBodyInfo("$(PROJECT_DIR)/../model/sample1.wrl")
    # set initial pose from sample/controller/SampleController/etc/Sample.pos
    hcf.seq_svc.setJointAngles(initial_pose, 2.0)
    hcf.seq_svc.waitInterpolation()
    hrpsys_version = hcf.seq.ref.get_component_profile().version
    print("hrpsys_version = %s"%hrpsys_version)

def demo ():
    init()
    from distutils.version import StrictVersion
    if StrictVersion(hrpsys_version) >= StrictVersion('315.5.0'):
        demoTestAllLimitTables()
        demoPositionLimit()
        demoVelocityLimit()
        demoErrorLimit()

def demoTestAllLimitTables():
    print("1. demo all jointLimitTables", file=sys.stderr)
    for table_idx in range(len(limit_table_list)/6):
        testLimitTables(table_idx, True, 5)

def rad2deg (ang):
    return 180.0*ang/3.14159

def deg2rad (ang):
    return 3.14159*ang/180.0

def getJointLimitTableInfo (table_idx=0):
    self_joint_name = limit_table_list[0+table_idx*6].replace(' ', '')
    target_joint_name = limit_table_list[1+table_idx*6].replace(' ', '')
    self_jointId = [x for x in bodyinfo._get_links() if x.name == self_joint_name][0].jointId
    target_jointId = [x for x in bodyinfo._get_links() if x.name == target_joint_name][0].jointId
    target_llimit=float(limit_table_list[2+table_idx*6])
    target_ulimit=float(limit_table_list[3+table_idx*6])
    self_llimits=[float(x) for x in limit_table_list[4+table_idx*6].split(",")]
    self_ulimits=[float(x) for x in limit_table_list[5+table_idx*6].split(",")]
    return [self_joint_name, target_joint_name,
            self_jointId, target_jointId,
            target_llimit, target_ulimit,
            self_llimits, self_ulimits]

def testLimitTables (table_idx=0, debug=True, loop_mod=1):
    [self_joint_name, target_joint_name,
     self_jointId, target_jointId,
     target_llimit, target_ulimit,
     self_llimits, self_ulimits] = getJointLimitTableInfo(table_idx)
    lret = testOneLimitTable(self_jointId, target_jointId, self_llimits, target_llimit, target_ulimit, -1, debug, loop_mod)
    uret = testOneLimitTable(self_jointId, target_jointId, self_ulimits, target_llimit, target_ulimit, 1, debug, loop_mod)
    print("lower limit check(", self_joint_name, ",", target_joint_name,")=", lret, file=sys.stderr)
    print("upper limit check(", self_joint_name, ",", target_joint_name,")=", uret, file=sys.stderr)
    assert(lret)
    assert(uret)

def testOneLimitTable (self_jointId, target_jointId, limit_table, target_llimit, target_ulimit, angle_violation, debug=True, loop_mod=1, thre=1e-2):
    tmp_pose=[x for x in initial_pose]
    ret=[]
    for idx in range(int(target_ulimit-target_llimit+1)):
        if idx%loop_mod != 0: # skip if loop_mod is specified
            continue
        if debug:
            print("idx=",idx, end=' ')
        # A-1. set safe joint
        tmp_pose[target_jointId]=deg2rad(target_llimit + idx);
        tmp_pose[self_jointId]=deg2rad(limit_table[idx]);
        if idx == 0:
            hcf.seq_svc.setJointAngles(tmp_pose, 1);
        else:
            hcf.seq_svc.setJointAngles(tmp_pose, 0.01);
        hcf.seq_svc.waitInterpolation()
        # A-2. check joint limit is not violated
        #   Dummy setJointAngles to wait for command joint angles are static
        hcf.seq_svc.setJointAngles(tmp_pose, 0.01);hcf.seq_svc.waitInterpolation();
        #   Use RobotHardware's command as SoftErrorLimiter joint angle output
        el_out1 = hcf.getActualState().command
        ret1 = abs(rad2deg(el_out1[self_jointId])-limit_table[idx]) < thre and abs(rad2deg(el_out1[target_jointId])- (target_llimit + idx)) < thre
        # B-1. set violated joint
        tmp_pose[self_jointId]=deg2rad(limit_table[idx]+angle_violation);
        hcf.seq_svc.setJointAngles(tmp_pose, 0.01);
        hcf.seq_svc.waitInterpolation()
        # B-2. check joint limit is not violated
        #   Dummy setJointAngles to wait for command joint angles are static
        hcf.seq_svc.setJointAngles(tmp_pose, 0.01);hcf.seq_svc.waitInterpolation()
        #   Use RobotHardware's command as SoftErrorLimiter joint angle output
        el_out2 = hcf.getActualState().command
        ret2 = abs(rad2deg(el_out2[self_jointId]) - limit_table[int(round(rad2deg(el_out2[target_jointId])-target_llimit))]) < thre # Check self and target is on limit table
        ret2 = ret2 and abs(el_out2[self_jointId] - (limit_table[idx]+angle_violation)) > thre # Check result value is not violated value
        # C. results
        if debug:
            print(" ret = (", ret1, ",", ret2,")")
            print("  self=(o1=", rad2deg(el_out1[self_jointId]), ", o2=", rad2deg(el_out2[self_jointId]), ", limit=", limit_table[idx], ") ", " target=(o1=", rad2deg(el_out1[target_jointId]), ", o2=", rad2deg(el_out2[target_jointId]), ", limit=", target_llimit + idx, ") [deg]")
        ret.append(ret1);
        ret.append(ret2);
        hcf.seq_svc.waitInterpolation()
    hcf.seq_svc.setJointAngles(initial_pose, 1);
    hcf.seq_svc.waitInterpolation()
    return all(ret)

def setAndCheckJointLimit (joint_name):
    print("  ", joint_name, file=sys.stderr)
    # ulimit check
    link_info=[x for x in bodyinfo._get_links() if x.name==joint_name][0]
    hcf.seq_svc.setJointAngle(joint_name, math.radians(1)+link_info.ulimit[0], 1)
    hcf.waitInterpolation()
    #   Dummy setJointAngles to wait for command joint angles are static
    hcf.seq_svc.setJointAngle(joint_name, math.radians(1)+link_info.ulimit[0], 0.01);hcf.waitInterpolation()
    #   Use RobotHardware's command as SoftErrorLimiter joint angle output
    tmppose = hcf.getActualState().command
    ret = tmppose[link_info.jointId] <= link_info.ulimit[0]
    print("    ulimit = ", ret, "(elout=", tmppose[link_info.jointId], ", limit=", link_info.ulimit[0], ")", file=sys.stderr)
    assert(ret)
    # llimit check
    hcf.seq_svc.setJointAngle(joint_name, math.radians(-1)+link_info.llimit[0], 1)
    hcf.waitInterpolation()
    #   Dummy setJointAngles to wait for command joint angles are static
    hcf.seq_svc.setJointAngle(joint_name, math.radians(-1)+link_info.llimit[0], 0.01);hcf.waitInterpolation()
    #   Use RobotHardware's command as SoftErrorLimiter joint angle output
    tmppose = hcf.getActualState().command
    ret = tmppose[link_info.jointId] >= link_info.llimit[0]
    print("    llimit = ", ret, "(elout=", tmppose[link_info.jointId], ", limit=", link_info.llimit[0], ")", file=sys.stderr)
    assert(ret)
    # go to initial
    hcf.seq_svc.setJointAngles(initial_pose, 1.0)
    hcf.waitInterpolation()

def demoPositionLimit():
    print("2. Check Position limit", file=sys.stderr)
    setAndCheckJointLimit('LARM_WRIST_Y')
    setAndCheckJointLimit('LARM_WRIST_P')
    setAndCheckJointLimit('LARM_SHOULDER_P')

def setAndCheckJointVelocityLimit (joint_name, thre=1e-5, dt=0.002):
    link_info=[x for x in bodyinfo._get_links() if x.name==joint_name][0]
    # lvlimit and uvlimit existence check
    if not(len(link_info.lvlimit) == 1 and len(link_info.uvlimit) == 1):
        print("  ", joint_name, " test neglected because no lvlimit and uvlimit are found.", file=sys.stderr)
        return
    for is_upper_limit in [True, False]: # uvlimit or lvlimit
        print("  ", joint_name, ", uvlimit" if is_upper_limit else ", lvlimit", file=sys.stderr)
        # Disable error limit for checking vel limit
        hcf.el_svc.setServoErrorLimit("all", 100000)
        # Test motion and logging
        hcf.clearLog()
        target_angle = (math.degrees( link_info.ulimit[0] if is_upper_limit else link_info.llimit[0] )*0.99) # 0.99 is margin
        vel_limit = link_info.uvlimit[0] if is_upper_limit else link_info.lvlimit[0]
        wait_time = abs(target_angle/math.degrees(vel_limit) * 1.1) # 1.1 is margin
        hcf.setJointAngle(joint_name, math.degrees(initial_pose[link_info.jointId]), 0.1)
        hcf.waitInterpolation()
        hcf.setJointAngle(joint_name, target_angle, 0.002)
        hcf.waitInterpolation()
        hcf.setJointAngle(joint_name, target_angle, wait_time) # Wait for finishing of joint motion
        hcf.waitInterpolation()
        #   Dummy setJointAngles to wait for command joint angles are static
        hcf.setJointAngle(joint_name, target_angle, 0.01);hcf.waitInterpolation()
        hcf.saveLog("/tmp/test-samplerobot-el-vel-check")
        # Check whether joint angle is reached
        #   Use RobotHardware's command as SoftErrorLimiter joint angle output
        reach_angle = math.degrees(hcf.getActualState().command[link_info.jointId])
        is_reached = abs(reach_angle - target_angle) < thre
        # Check actual velocity from Datalogger log
        poslist=[]
        for line in open("/tmp/test-samplerobot-el-vel-check.SampleRobot(Robot)0_q", "r"):
            poslist.append(float(line.split(" ")[link_info.jointId+1]))
        tmp = list(map(lambda x,y : x-y, poslist[1:], poslist[0:-1]))
        max_ret_vel = max(tmp)/dt if is_upper_limit else min(tmp)/dt
        is_vel_limited = abs(max_ret_vel - vel_limit) < thre
        # Enable error limit by reverting limit value and reset joint angle
        hcf.el_svc.setServoErrorLimit("all", (0.2-0.02))
        hcf.setJointAngle(joint_name, math.degrees(initial_pose[link_info.jointId]), 0.5)
        hcf.waitInterpolation()
        # Check flags and print
        print("    is_reached =", is_reached, ", is_vel_limited =", is_vel_limited, end=' ', file=sys.stderr)
        print(", target_angle =", target_angle, "[deg], reach_angle =", reach_angle, "[deg], max_ret_vel =", max_ret_vel, "[rad/s], vel_limit =", vel_limit, "[rad/s]", file=sys.stderr)
        assert(is_reached and is_vel_limited)

def demoVelocityLimit():
    print("3. Check Velocity limit", file=sys.stderr)
    setAndCheckJointVelocityLimit('LARM_WRIST_Y')
    setAndCheckJointVelocityLimit('LARM_WRIST_P')

def setAndCheckJointErrorLimit (joint_name, thre=1e-5):
    link_info=[x for x in bodyinfo._get_links() if x.name==joint_name][0]
    for is_upper_limit in [True, False]: # uvlimit or lvlimit
        print("  ", joint_name, ", uvlimit" if is_upper_limit else ", lvlimit", file=sys.stderr)
        # Disable error limit for checking vel limit
        error_limit = 0.002 if is_upper_limit else -0.002 # [rad]
        hcf.el_svc.setServoErrorLimit("all", abs(error_limit))
        # Test motion and logging
        hcf.clearLog()
        target_angle = 3.0 if is_upper_limit else -3.0 # [deg]
        wait_time = 0.2
        hcf.setJointAngle(joint_name, math.degrees(initial_pose[link_info.jointId]), 0.1)
        hcf.waitInterpolation()
        hcf.setJointAngle(joint_name, target_angle, 0.002)
        hcf.waitInterpolation()
        hcf.setJointAngle(joint_name, target_angle, wait_time) # Wait for finishing of joint motion
        hcf.waitInterpolation()
        #   Dummy setJointAngles to wait for command joint angles are static
        hcf.setJointAngle(joint_name, target_angle, 0.01);hcf.waitInterpolation()
        hcf.saveLog("/tmp/test-samplerobot-el-err-check")
        # Check whether joint angle is reached
        #   Use RobotHardware's command as SoftErrorLimiter joint angle output
        reach_angle = math.degrees(hcf.getActualState().command[link_info.jointId])
        is_reached = abs(reach_angle - target_angle) < thre
        # Check actual velocity from Datalogger log
        poslist=[]
        for line in open("/tmp/test-samplerobot-el-err-check.SampleRobot(Robot)0_q", "r"):
            poslist.append(float(line.split(" ")[link_info.jointId+1]))
        refposlist=[]
        for line in open("/tmp/test-samplerobot-el-err-check.el_q", "r"):
            refposlist.append(float(line.split(" ")[link_info.jointId+1]))
        tmp = list(map(lambda x,y : x-y, refposlist, poslist))
        max_ret_err = max(tmp) if is_upper_limit else min(tmp)
        is_err_limited = abs(max_ret_err - error_limit) < thre
        # Enable error limit by reverting limit value and reset joint angle
        hcf.el_svc.setServoErrorLimit("all", (0.2-0.02))
        hcf.setJointAngle(joint_name, math.degrees(initial_pose[link_info.jointId]), 0.5)
        hcf.waitInterpolation()
        # Check flags and print
        print("    is_reached =", is_reached, ", is_err_limited =", is_err_limited, end=' ', file=sys.stderr)
        print(", target_angle =", target_angle, "[deg], reach_angle =", reach_angle, "[deg], max_ret_err =", max_ret_err, "[rad], err_limit =", error_limit, "[rad]", file=sys.stderr)
        assert(is_reached and is_err_limited)

def demoErrorLimit():
    print("4. Check Error limit", file=sys.stderr)
    setAndCheckJointErrorLimit('LARM_WRIST_Y')
    setAndCheckJointErrorLimit('LARM_WRIST_P')


if __name__ == '__main__':
    demo()

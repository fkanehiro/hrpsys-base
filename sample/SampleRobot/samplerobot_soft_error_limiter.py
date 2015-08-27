#!/usr/bin/env python

from hrpsys.hrpsys_config import *
from hrpsys import OpenHRP
from hrpsys import rtm

from subprocess import check_output

# Tempolarily remove tc which is limit position range
def getRTCList ():
    return filter(lambda x : x[0]!='tc', hcf.getRTCListUnstable())

def init ():
    global hcf, initial_pose, limit_table_list, bodyinfo, hrpsys_version
    hcf = HrpsysConfigurator()
    hcf.getRTCList = getRTCList
    hcf.init ("SampleRobot(Robot)0", "$(PROJECT_DIR)/../model/sample1.wrl")
    initial_pose = [-7.779e-005,  -0.378613,  -0.000209793,  0.832038,  -0.452564,  0.000244781,  0.31129,  -0.159481,  -0.115399,  -0.636277,  0,  0,  0,  -7.77902e-005,  -0.378613,  -0.000209794,  0.832038,  -0.452564,  0.000244781,  0.31129,  0.159481,  0.115399,  -0.636277,  0,  0,  0,  0,  0,  0]
    # load joint limit table from conf file
    HRPSYS_DIR=check_output(['pkg-config', 'hrpsys-base', '--variable=prefix']).rstrip()
    f=open("{}/share/hrpsys/samples/SampleRobot/SampleRobot.500.el.conf".format(HRPSYS_DIR))
    limit_table_str=filter(lambda x : x.find("joint_limit_table") > -1 , f.readlines())[0]
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
    if hrpsys_version >= '315.5.0':
        demoTestAllLimitTables()
    demoPositionLimit()

def demoTestAllLimitTables():
    print >> sys.stderr, "1. demo all jointLimitTables"
    for table_idx in range(len(limit_table_list)/6):
        testLimitTables(table_idx, True, 5)

def rad2deg (ang):
    return 180.0*ang/3.14159

def deg2rad (ang):
    return 3.14159*ang/180.0

def getJointLimitTableInfo (table_idx=0):
    self_joint_name = limit_table_list[0+table_idx*6].replace(' ', '')
    target_joint_name = limit_table_list[1+table_idx*6].replace(' ', '')
    self_jointId = filter( lambda x : x.name == self_joint_name, bodyinfo._get_links())[0].jointId
    target_jointId = filter( lambda x : x.name == target_joint_name, bodyinfo._get_links())[0].jointId
    target_llimit=float(limit_table_list[2+table_idx*6])
    target_ulimit=float(limit_table_list[3+table_idx*6])
    self_llimits=map(lambda x : float(x), limit_table_list[4+table_idx*6].split(","))
    self_ulimits=map(lambda x : float(x), limit_table_list[5+table_idx*6].split(","))
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
    print >> sys.stderr, "lower limit check(", self_joint_name, ",", target_joint_name,")=", lret
    print >> sys.stderr, "upper limit check(", self_joint_name, ",", target_joint_name,")=", uret
    assert(lret)
    assert(uret)

def testOneLimitTable (self_jointId, target_jointId, limit_table, target_llimit, target_ulimit, angle_violation, debug=True, loop_mod=1):
    tmp_pose=map(lambda x : x, initial_pose)
    ret=[]
    for idx in range(int(target_ulimit-target_llimit+1)):
        if idx%loop_mod != 0: # skip if loop_mod is specified
            continue
        if debug:
            print "idx=",idx,
        # A-1. set safe joint
        tmp_pose[target_jointId]=deg2rad(target_llimit + idx);
        tmp_pose[self_jointId]=deg2rad(limit_table[idx]);
        if idx == 0:
            hcf.seq_svc.setJointAngles(tmp_pose, 1);
        else:
            hcf.seq_svc.setJointAngles(tmp_pose, 0.01);
        hcf.seq_svc.waitInterpolation()
        # A-2. check joint limit is not violated
        el_out1 = rtm.readDataPort(hcf.el.port("q")).data
        ret1 = abs(rad2deg(el_out1[self_jointId])-limit_table[idx]) < 1e-3 and abs(rad2deg(el_out1[target_jointId])- (target_llimit + idx)) < 1e-3
        # B-1. set violated joint
        tmp_pose[self_jointId]=deg2rad(limit_table[idx]+angle_violation);
        hcf.seq_svc.setJointAngles(tmp_pose, 0.01);
        hcf.seq_svc.waitInterpolation()
        # B-2. check joint limit is not violated
        el_out2=rtm.readDataPort(hcf.el.port("q")).data
        ret2 = abs(rad2deg(el_out2[self_jointId])-limit_table[idx]) < 1e-3 and abs(rad2deg(el_out2[target_jointId])- (target_llimit + idx)) < 1e-3
        # C. results
        if debug:
            print " ret = (", ret1, ",", ret2,")"
            print "  self=(o1=", rad2deg(el_out1[self_jointId]), ", o2=", rad2deg(el_out2[self_jointId]), ", limit=", limit_table[idx], ") ", " target=(o1=", rad2deg(el_out1[target_jointId]), ", o2=", rad2deg(el_out2[target_jointId]), ", limit=", target_llimit + idx, ") [deg]"
        ret.append(ret1);
        ret.append(ret2);
        hcf.seq_svc.waitInterpolation()
    hcf.seq_svc.setJointAngles(initial_pose, 1);
    hcf.seq_svc.waitInterpolation()
    return all(ret)

def setAndCheckJointLimit (joint_name):
    mdlldr=hcf.getBodyInfo("$(PROJECT_DIR)/../model/sample1.wrl")
    print >> sys.stderr, "  ", joint_name
    # ulimit check
    link_info=filter(lambda x : x.name==joint_name, mdlldr._get_links())[0]
    hcf.seq_svc.setJointAngle(joint_name, math.radians(1)+link_info.ulimit[0], 1)
    hcf.waitInterpolation()
    ret = rtm.readDataPort(hcf.el.port("q")).data[link_info.jointId] < link_info.ulimit[0]
    print >> sys.stderr, "    ulimit = ", ret, "(elout=", rtm.readDataPort(hcf.el.port("q")).data[link_info.jointId], ", limit=", link_info.ulimit[0], ")"
    assert(ret)
    # llimit check
    hcf.seq_svc.setJointAngle(joint_name, math.radians(-1)+link_info.llimit[0], 1)
    hcf.waitInterpolation()
    ret = rtm.readDataPort(hcf.el.port("q")).data[link_info.jointId] > link_info.llimit[0]
    print >> sys.stderr, "    llimit = ", ret, "(elout=", rtm.readDataPort(hcf.el.port("q")).data[link_info.jointId], ", limit=", link_info.llimit[0], ")"
    assert(ret)
    # go to initial
    hcf.seq_svc.setJointAngles(initial_pose, 1.0)
    hcf.waitInterpolation()

def demoPositionLimit():
    print >> sys.stderr, "2. Position limit"
    setAndCheckJointLimit('LARM_WRIST_Y')
    setAndCheckJointLimit('LARM_WRIST_P')
    setAndCheckJointLimit('LARM_SHOULDER_P')

if __name__ == '__main__':
    demo()

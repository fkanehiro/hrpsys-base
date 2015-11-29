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

def init ():
    global hcf, initial_pose, hrpsys_version
    hcf = HrpsysConfigurator()
    hcf.getRTCList = hcf.getRTCListUnstable
    hcf.init ("SampleSpecialJointRobot(Robot)0", "$(PROJECT_DIR)/../model/sample_special_joint_robot.wrl")
    initial_pose = [0.0, -0.20944, -0.20944, 0.0, 0.523599, 0.523599, -0.314159, -0.314159, 0.0, 0.0,
                    0.0, -0.20944, -0.20944, 0.0, 0.523599, 0.523599, -0.314159, -0.314159, 0.0, 0.0,
                    0.0,      0.0,      0.0]
    hcf.seq_svc.setJointAngles(initial_pose, 2.0)
    hcf.waitInterpolation()
    hcf.startAutoBalancer()
    hrpsys_version = hcf.seq.ref.get_component_profile().version
    print("hrpsys_version = %s"%hrpsys_version)

def checkActualBaseAttitude():
    rpy = rtm.readDataPort(hcf.rh.port("WAIST")).data.orientation
    ret = math.degrees(rpy.r) < 0.1 and math.degrees(rpy.p) < 0.1
    print >> sys.stderr, "  actual base rpy = ", ret, "(", rpy, ")"
    assert (ret)
    return ret

def demoGaitGeneratorNoToeHeelContact():
    print >> sys.stderr, "1. Do not use toe heel contact"
    hcf.abc_svc.goPos(0.3, 0, 0);
    hcf.abc_svc.waitFootSteps()
    checkActualBaseAttitude()
    print >> sys.stderr, "  No toe heel contact=>OK"

def demoGaitGeneratorToeHeelContact():
    print >> sys.stderr, "2. Use toe heel contact"
    ggp=hcf.abc_svc.getGaitGeneratorParam()[1];
    ggp.toe_pos_offset_x = 1e-3*182.0;
    ggp.heel_pos_offset_x = 1e-3*-72.0;
    ggp.toe_zmp_offset_x = 1e-3*182.0;
    ggp.heel_zmp_offset_x = 1e-3*-72.0;
    ggp.toe_angle = 35;
    ggp.heel_angle = 10;
    ggp.use_toe_joint = False;
    hcf.abc_svc.setGaitGeneratorParam(ggp);
    hcf.abc_svc.goPos(0.3, 0, 0);
    hcf.abc_svc.waitFootSteps()
    ggp.toe_angle = 0;
    ggp.heel_angle = 0;
    hcf.abc_svc.setGaitGeneratorParam(ggp);
    checkActualBaseAttitude()
    print >> sys.stderr, "  Toe heel contact=>OK"

def demoGaitGeneratorToeHeelContactWithToeJoint():
    print >> sys.stderr, "3. Use toe heel contact with toe joint"
    ggp=hcf.abc_svc.getGaitGeneratorParam()[1];
    ggp.toe_pos_offset_x = 1e-3*182.0;
    ggp.heel_pos_offset_x = 1e-3*-72.0;
    ggp.toe_zmp_offset_x = 1e-3*182.0;
    ggp.heel_zmp_offset_x = 1e-3*-72.0;
    ggp.toe_angle = 35;
    ggp.heel_angle = 10;
    ggp.use_toe_joint = True;
    hcf.abc_svc.setGaitGeneratorParam(ggp);
    hcf.abc_svc.goPos(0.3, 0, 0);
    hcf.abc_svc.waitFootSteps()
    ggp.toe_angle = 0;
    ggp.heel_angle = 0;
    hcf.abc_svc.setGaitGeneratorParam(ggp);
    checkActualBaseAttitude()
    print >> sys.stderr, "  Toe heel contact with toe joint =>OK"


def demo():
    init()
    demoGaitGeneratorNoToeHeelContact()
    demoGaitGeneratorToeHeelContact()
    demoGaitGeneratorToeHeelContactWithToeJoint()

if __name__ == '__main__':
    demo()

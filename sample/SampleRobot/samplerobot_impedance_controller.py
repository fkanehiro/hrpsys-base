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
    global hcf, hrpsys_version
    hcf = HrpsysConfigurator()
    hcf.getRTCList = hcf.getRTCListUnstable
    hcf.init ("SampleRobot(Robot)0", "$(PROJECT_DIR)/../model/sample1.wrl")
    hrpsys_version = hcf.seq.ref.get_component_profile().version
    print("hrpsys_version = %s"%hrpsys_version)

def demo():
    init()
    # set initial pose from sample/controller/SampleController/etc/Sample.pos
    initial_pose = [-7.779e-005,  -0.378613,  -0.000209793,  0.832038,  -0.452564,  0.000244781,  0.31129,  -0.159481,  -0.115399,  -0.636277,  0,  0,  0,  -7.77902e-005,  -0.378613,  -0.000209794,  0.832038,  -0.452564,  0.000244781,  0.31129,  0.159481,  0.115399,  -0.636277,  0,  0,  0,  0,  0,  0]
    hcf.seq_svc.setJointAngles(initial_pose, 2.0)
    hcf.seq_svc.waitInterpolation()

    # 0. startImpedance + stopImpedance python interface
    print >> sys.stderr, "0. startImpedance + stopImpedance python interface"
    hcf.startImpedance("rarm")
    hcf.startImpedance("larm")
    hcf.stopImpedance("larm")
    hcf.stopImpedance("rarm")
    if hrpsys_version < '315.5.0':
        return

    # 1. Getter check
    print >> sys.stderr, "1. Getter check"
    ret1=hcf.ic_svc.getImpedanceControllerParam("rarm")
    if ret1[0]:
        print >> sys.stderr, "  getImpedanceControllerParam => OK"
    # 2. Setter check
    print >> sys.stderr, "2. Setter check"
    ret1[1].K_r=1.0
    ret1[1].D_r=2.0
    ret1[1].M_r=0.2
    ret2=hcf.ic_svc.setImpedanceControllerParam("rarm", ret1[1])
    ret3=hcf.ic_svc.getImpedanceControllerParam("rarm")
    if ret2:
        print >> sys.stderr, "  setImpedanceControllerParam => OK"
    # 3. Start impedance
    print >> sys.stderr, "3. Start impedance"
    ret4=hcf.ic_svc.startImpedanceController("rarm")
    if ret4:
        print >> sys.stderr, "  startImpedanceController => OK"
    # 4. Set ref force and moment
    print >> sys.stderr, "4. Set ref force and moment"
    hcf.seq_svc.setWrenches([0,0,0,0,0,0,
                             0,0,0,0,0,0,
                             0,0,0,0,0,0,
                             20,10,-10,0,0,0,], 2.0);
    hcf.seq_svc.waitInterpolation();
    time.sleep(2)
    hcf.seq_svc.setWrenches([0,0,0,0,0,0,
                             0,0,0,0,0,0,
                             0,0,0,0,0,0,
                             0,0,0,0,0,0,], 2.0);
    hcf.seq_svc.waitInterpolation();
    time.sleep(2)
    hcf.seq_svc.setWrenches([0,0,0,0,0,0,
                             0,0,0,0,0,0,
                             0,0,0,0,0,0,
                             0,0,0,0.1,-0.1,0.1], 2.0);
    hcf.seq_svc.waitInterpolation();
    time.sleep(2)
    hcf.seq_svc.setWrenches([0,0,0,0,0,0,
                             0,0,0,0,0,0,
                             0,0,0,0,0,0,
                             0,0,0,0,0,0], 2.0);
    hcf.seq_svc.waitInterpolation();
    time.sleep(2)
    # 5. Stop impedance
    print >> sys.stderr, "5. Stop impedance"
    ret5=hcf.ic_svc.stopImpedanceController("rarm")
    if ret5:
        print >> sys.stderr, "  stopImpedanceController => OK"
    # 6. Arm tracking check
    print >> sys.stderr, "6. Arm tracking check"
    hcf.ic_svc.startImpedanceController("rarm")
    hcf.setJointAngle("RARM_ELBOW", -40.0, 0.5);
    hcf.waitInterpolation()
    hcf.setJointAngle("RARM_ELBOW", -70.0, 0.5);
    hcf.waitInterpolation()
    # 7. World frame check
    if hcf.kinematics_only_mode:
        print >> sys.stderr, "7. World frame check"
        # tempolarily set use_sh_base_pos_rpy
        icp=hcf.ic_svc.getImpedanceControllerParam("rarm")[1]
        icp.use_sh_base_pos_rpy = True
        hcf.ic_svc.setImpedanceControllerParam("rarm", icp)
        # test
        hcf.seq_svc.setJointAngles(initial_pose, 2.0)
        hcf.seq_svc.waitInterpolation()
        hcf.setJointAngle("RLEG_ANKLE_P",40, 1);
        hcf.waitInterpolation()
        hcf.setJointAngle("RLEG_ANKLE_P",-40, 1);
        hcf.waitInterpolation()
        hcf.seq_svc.setJointAngles(initial_pose, 2.0)
        hcf.seq_svc.waitInterpolation()
    else:
        print >> sys.stderr, "7. World frame check is not executed in non-kinematics-only-mode"
    # 8. World frame ref-force check
    if hcf.kinematics_only_mode:
        print >> sys.stderr, "8. World frame ref-force check"
        # tempolarily set use_sh_base_pos_rpy
        icp=hcf.ic_svc.getImpedanceControllerParam("rarm")[1]
        icp.use_sh_base_pos_rpy = True
        hcf.ic_svc.setImpedanceControllerParam("rarm", icp)
        # test
        hcf.seq_svc.setBaseRpy([0,0,0], 1.0);
        hcf.seq_svc.waitInterpolation();
        hcf.seq_svc.setWrenches([0,0,0,0,0,0,
                                 0,0,0,0,0,0,
                                 0,0,0,0,0,0,
                                 -40,0,0,0,0,0], 2.0);
        hcf.seq_svc.waitInterpolation();
        hcf.seq_svc.setWrenches([0,0,0,0,0,0,
                                 0,0,0,0,0,0,
                                 0,0,0,0,0,0,
                                 0,0,0,0,0,0], 1.0);
        hcf.seq_svc.waitInterpolation();
        hcf.seq_svc.setBaseRpy([0,45*3.14159/180,0], 1.0);
        hcf.seq_svc.waitInterpolation();
        hcf.seq_svc.setWrenches([0,0,0,0,0,0,
                                 0,0,0,0,0,0,
                                 0,0,0,0,0,0,
                                 -40,0,0,0,0,0], 2.0);
        hcf.seq_svc.waitInterpolation();
        hcf.seq_svc.setWrenches([0,0,0,0,0,0,
                                 0,0,0,0,0,0,
                                 0,0,0,0,0,0,
                                 0,0,0,0,0,0], 1.0);
        hcf.seq_svc.waitInterpolation();
        hcf.seq_svc.setBaseRpy([0,0,0], 1.0);
        hcf.seq_svc.waitInterpolation();
    else:
        print >> sys.stderr, "8. World frame ref-force check is not executed in non-kinematics-only-mode"

if __name__ == '__main__':
    demo()

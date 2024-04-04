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

def init ():
    global hcf, hrpsys_version
    hcf = HrpsysConfigurator()
    hcf.getRTCList = hcf.getRTCListUnstable
    hcf.init ("SampleRobot(Robot)0", "$(PROJECT_DIR)/../model/sample1.wrl")
    hrpsys_version = hcf.seq.ref.get_component_profile().version
    print("hrpsys_version = %s"%hrpsys_version)
    # set initial pose from sample/controller/SampleController/etc/Sample.pos
    initial_pose = [-7.779e-005,  -0.378613,  -0.000209793,  0.832038,  -0.452564,  0.000244781,  0.31129,  -0.159481,  -0.115399,  -0.636277,  0,  0,  0,  -7.77902e-005,  -0.378613,  -0.000209794,  0.832038,  -0.452564,  0.000244781,  0.31129,  0.159481,  0.115399,  -0.636277,  0,  0,  0,  0,  0,  0]
    hcf.seq_svc.setJointAngles(initial_pose, 2.0)
    hcf.seq_svc.waitInterpolation()

def demoStartStopIMP ():
    # 0. startImpedance + stopImpedance python interface
    print("0. startImpedance + stopImpedance python interface", file=sys.stderr)
    hcf.startImpedance("rarm")
    hcf.startImpedance("larm")
    hcf.stopImpedance("larm")
    hcf.stopImpedance("rarm")

def demoGetImpedanceControllerParam ():
    # 1. Getter check
    print("1. Getter check", file=sys.stderr)
    all_get_ret = []
    for limb in ["rarm", "larm"]:
        all_get_ret.append(hcf.ic_svc.getImpedanceControllerParam(limb)[0])
    print("  all_get_ret = ", all_get_ret, file=sys.stderr)
    assert(all(all_get_ret))
    print("  getImpedanceControllerParam => OK", file=sys.stderr)

def demoSetImpedanceControllerParam ():
    # 2. Setter check
    print("2. Setter check", file=sys.stderr)
    all_set_ret = []
    all_value_ret = []
    for limb in ["rarm", "larm"]:
        [ret1, icp1]=hcf.ic_svc.getImpedanceControllerParam(limb)
        icp1.K_r=1.0
        icp1.D_r=2.0
        icp1.M_r=0.2
        all_set_ret.append(hcf.ic_svc.setImpedanceControllerParam(limb, icp1))
        [ret2, icp2]=hcf.ic_svc.getImpedanceControllerParam(limb)
        all_value_ret.append((icp1.M_r == icp2.M_r) and (icp1.D_r == icp2.D_r) and (icp1.K_r == icp2.K_r))
    print("  all_set_ret = ", all_set_ret, ", all_value_ret = ", all_value_ret, file=sys.stderr)
    assert(all(all_set_ret) and all(all_value_ret))
    print("  setImpedanceControllerParam => OK", file=sys.stderr)

def demoStartImpedanceController ():
    # 3. Start impedance
    print("3. Start impedance", file=sys.stderr)
    all_start_ret = []
    all_mode_ret = []
    # start
    for limb in ["rarm", "larm"]:
        [ret, icp]=hcf.ic_svc.getImpedanceControllerParam(limb)
        all_start_ret.append(hcf.ic_svc.startImpedanceControllerNoWait(limb))
        all_mode_ret.append(icp.controller_mode == OpenHRP.ImpedanceControllerService.MODE_IDLE)
    # wait and check
    for limb in ["rarm", "larm"]:
        hcf.ic_svc.waitImpedanceControllerTransition(limb)
        [ret, icp]=hcf.ic_svc.getImpedanceControllerParam(limb)
        all_mode_ret.append(icp.controller_mode == OpenHRP.ImpedanceControllerService.MODE_IMP)
    # "already start" check
    for limb in ["rarm", "larm"]:
        all_start_ret.append(not hcf.ic_svc.startImpedanceControllerNoWait(limb))
    print("  all_start_ret = ", all_start_ret, ", all_mode_ret = ", all_mode_ret, file=sys.stderr)
    assert(all(all_start_ret) and all(all_mode_ret))
    print("  startImpedanceController => OK", file=sys.stderr)

def demoSetRefForce ():
    # 4. Set ref force and moment
    print("4. Set ref force and moment", file=sys.stderr)
    hcf.seq_svc.setWrenches([0,0,0,0,0,0,
                             0,0,0,0,0,0,
                             0,0,0,0,0,0,
                             20,10,-10,0,0,0,], 1.0);
    hcf.seq_svc.waitInterpolation();
    time.sleep(2)
    hcf.seq_svc.setWrenches([0,0,0,0,0,0,
                             0,0,0,0,0,0,
                             0,0,0,0,0,0,
                             0,0,0,0,0,0,], 1.0);
    hcf.seq_svc.waitInterpolation();
    time.sleep(2)
    hcf.seq_svc.setWrenches([0,0,0,0,0,0,
                             0,0,0,0,0,0,
                             0,0,0,0,0,0,
                             0,0,0,0.1,-0.1,0.1], 1.0);
    hcf.seq_svc.waitInterpolation();
    time.sleep(2)
    hcf.seq_svc.setWrenches([0,0,0,0,0,0,
                             0,0,0,0,0,0,
                             0,0,0,0,0,0,
                             0,0,0,0,0,0], 1.0);
    hcf.seq_svc.waitInterpolation();
    time.sleep(2)

def demoStopImpedanceController ():
    # 5. Stop impedance
    print("5. Stop impedance", file=sys.stderr)
    all_stop_ret = []
    all_mode_ret = []
    # stop
    for limb in ["rarm", "larm"]:
        all_stop_ret.append(hcf.ic_svc.stopImpedanceControllerNoWait(limb))
    # wait and check
    for limb in ["rarm", "larm"]:
        hcf.ic_svc.waitImpedanceControllerTransition(limb)
        [ret, icp]=hcf.ic_svc.getImpedanceControllerParam(limb)
        all_mode_ret.append(icp.controller_mode == OpenHRP.ImpedanceControllerService.MODE_IDLE)
    # "already stop" check
    for limb in ["rarm", "larm"]:
        all_stop_ret.append(not hcf.ic_svc.stopImpedanceControllerNoWait(limb))
    print("  all_stop_ret = ", all_stop_ret, ", all_mode_ret = ", all_mode_ret, file=sys.stderr)
    assert(all(all_stop_ret) and all(all_mode_ret))
    print("  stopImpedanceController => OK", file=sys.stderr)

def demoArmTrackingCheck ():
    # 6. Arm tracking check
    print("6. Arm tracking check", file=sys.stderr)
    hcf.ic_svc.startImpedanceController("rarm")
    hcf.setJointAngle("RARM_ELBOW", -40.0, 0.5);
    hcf.waitInterpolation()
    hcf.setJointAngle("RARM_ELBOW", -70.0, 0.5);
    hcf.waitInterpolation()

def demoWorldFrameCheck ():
    # 7. World frame check
    if hcf.kinematics_only_mode:
        print("7. World frame check", file=sys.stderr)
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
        print("7. World frame check is not executed in non-kinematics-only-mode", file=sys.stderr)

def demoWorldFrameRefForceCheck ():
    # 8. World frame ref-force check
    if hcf.kinematics_only_mode:
        print("8. World frame ref-force check", file=sys.stderr)
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
                                 -40,0,0,0,0,0], 1.0);
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
                                 -40,0,0,0,0,0], 1.0);
        hcf.seq_svc.waitInterpolation();
        hcf.seq_svc.setWrenches([0,0,0,0,0,0,
                                 0,0,0,0,0,0,
                                 0,0,0,0,0,0,
                                 0,0,0,0,0,0], 1.0);
        hcf.seq_svc.waitInterpolation();
        hcf.seq_svc.setBaseRpy([0,0,0], 1.0);
        hcf.seq_svc.waitInterpolation();
    else:
        print("8. World frame ref-force check is not executed in non-kinematics-only-mode", file=sys.stderr)

def demoOCTDCheck ():
    # 1. Object Contact Turnaround Detector set param check
    print("1. Object Contact Turnaround Detector set param check", file=sys.stderr)
    ret9 = True
    detect_time_thre = 0.3
    start_time_thre=0.3
    for number_disturbance in [0, 1e-5, -1e-5]: # 1e-5 is smaller than dt
        octdp=hcf.octd_svc.getObjectContactTurnaroundDetectorParam()[1];
        octdp.detect_time_thre = detect_time_thre + number_disturbance
        octdp.start_time_thre = start_time_thre + number_disturbance
        hcf.octd_svc.setObjectContactTurnaroundDetectorParam(octdp);
        octdp2=hcf.octd_svc.getObjectContactTurnaroundDetectorParam()[1];
        print("  ", octdp2, file=sys.stderr)
        ret9 = ret9 and (octdp2.detect_time_thre == detect_time_thre and octdp2.start_time_thre == start_time_thre)
    assert(ret9)
    print("  => OK", file=sys.stderr)


def demo():
    init()

    demoStartStopIMP ()
    from distutils.version import StrictVersion
    if StrictVersion(hrpsys_version) < StrictVersion('315.5.0'):
        return

    demoGetImpedanceControllerParam()
    demoSetImpedanceControllerParam()
    demoStartImpedanceController()
    demoSetRefForce()
    demoStopImpedanceController()
    demoArmTrackingCheck()
    demoWorldFrameCheck()
    demoWorldFrameRefForceCheck()
    demoOCTDCheck()

if __name__ == '__main__':
    demo()

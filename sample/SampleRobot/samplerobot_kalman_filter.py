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
    global hcf, pitch_poses, roll_poses, yaw_poses, roll_pitch_poses, initial_pose, hrpsys_version
    hcf = HrpsysConfigurator()
    hcf.getRTCList = hcf.getRTCListUnstable
    hcf.init ("SampleRobot(Robot)0", "$(PROJECT_DIR)/../model/sample1.wrl")
    hcf.connectLoggerPort(hcf.kf, 'baseRpyCurrent')
    # initialize poses
    # pose1 = [0]*29
    # pose2 = [0]*29
    # pose2[3] = 20*3.14159/180.0
    # pose2[3+7+6] = 20*3.14159/180.0
    # pose2[4] = -10*3.14159/180.0
    # pose2[4+7+6] = -10*3.14159/180.0
    # pose3 = [0]*29
    # pose3[3] = -20*3.14159/180.0
    # pose3[3+7+6] = -20*3.14159/180.0
    # pose3[4] = 10*3.14159/180.0
    # pose3[4+7+6] = 10*3.14159/180.0
    # hcf.seq_svc.setJointAngles(pose1, 2.5)
    # hcf.seq_svc.waitInterpolation()
    initial_pose = [-7.779e-005,  -0.378613,  -0.000209793,  0.832038,  -0.452564,  0.000244781,  0.31129,  -0.159481,  -0.115399,  -0.636277,  0,  0,  0,  -7.77902e-005,  -0.378613,  -0.000209794,  0.832038,  -0.452564,  0.000244781,  0.31129,  0.159481,  0.115399,  -0.636277,  0,  0,  0,  0,  0,  0]
    # reset-pose+ 50 legup + 30deg pitch
    pitch_pose1=[0.000112,-0.553688,-0.00028,1.15739,-0.602847,-1.330313e-05,0.31129,-0.159481,-0.115399,-0.636277,0.0,0.0,0.0,0.000112,-0.553703,-0.00028,1.15739,-0.602833,-1.330597e-05,0.31129,0.159481,0.115399,-0.636277,0.0,0.0,0.0,0.0,0.0,0.0]
    pitch_pose2=[8.200401e-05,-0.980787,-0.000375,0.598284,-0.140235,-9.749026e-05,0.31129,-0.159481,-0.115399,-0.636277,0.0,0.0,0.0,8.206224e-05,-0.980716,-0.000375,0.598026,-0.140049,-9.749664e-05,0.31129,0.159481,0.115399,-0.636277,0.0,0.0,0.0,0.0,0.0,0.0]
    pitch_pose3=[0.000229,0.313052,-0.000276,0.779751,-0.568344,-3.437126e-05,0.31129,-0.159481,-0.115399,-0.636277,0.0,0.0,0.0,0.000229,0.313082,-0.000276,0.779709,-0.568332,-3.436731e-05,0.31129,0.159481,0.115399,-0.636277,0.0,0.0,0.0,0.0,0.0,0.0]
    pitch_poses = [pitch_pose1, pitch_pose2, pitch_pose3]
    # roll
    roll_pose1=[0.000112,-0.553688,-0.00028,1.15739,-0.602847,-1.330313e-05,0.31129,-0.159481,-0.115399,-0.636277,0.0,0.0,0.0,0.000112,-0.553703,-0.00028,1.15739,-0.602833,-1.330597e-05,0.31129,0.159481,0.115399,-0.636277,0.0,0.0,0.0,0.0,0.0,0.0]
    roll_pose2=[-0.475317,-0.568231,0.022036,1.1549,-0.595553,0.138268,0.31129,-0.159481,-0.115399,-0.636277,0.0,0.0,0.0,-0.447984,-0.378428,0.020283,0.753855,-0.383755,0.106576,0.31129,0.159481,0.115399,-0.636277,0.0,0.0,0.0,0.0,0.0,0.0]
    roll_pose3=[0.44835,-0.377793,-0.020814,0.752451,-0.38309,-0.106633,0.31129,-0.159481,-0.115399,-0.636277,0.0,0.0,0.0,0.475616,-0.567819,-0.022616,1.15389,-0.595074,-0.138377,0.31129,0.159481,0.115399,-0.636277,0.0,0.0,0.0,0.0,0.0,0.0]
    roll_poses = [roll_pose1, roll_pose2, roll_pose3]
    # yaw
    yaw_pose1=[-0.000113, -0.523213, -0.000225, 1.15585, -0.631772, 0.000245, 0.31129, -0.159481, -0.115399, -0.636277, 0.0, 0.0, 0.0, -0.000113, -0.523213, -0.000225, 1.15585, -0.631772, 0.000245, 0.31129, 0.159481, 0.115399, -0.636277, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    yaw_pose2=[-0.000113, -0.523213, -0.000225, 1.15585, -0.631772, 0.000245, 0.31129, -0.159481, -0.115399, -0.636277, 0.0, 0.0, 0.0, -0.000113, -0.523213, -0.000225, 1.15585, -0.631772, 0.000245, 0.31129, 0.159481, 0.115399, -0.636277, 0.0, 0.0, 0.0, 0.0, 0.0, 0.785398]
    yaw_pose3=[-0.000113, -0.523213, -0.000225, 1.15585, -0.631772, 0.000245, 0.31129, -0.159481, -0.115399, -0.636277, 0.0, 0.0, 0.0, -0.000113, -0.523213, -0.000225, 1.15585, -0.631772, 0.000245, 0.31129, 0.159481, 0.115399, -0.636277, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5708]
    yaw_poses = [yaw_pose1, yaw_pose2, yaw_pose3]
    # roll + pitch
    roll_pitch_pose1=[0.000111,-0.681981,-0.000333,1.41142,-0.728577,-7.604916e-05,0.31129,-0.159481,-0.115399,-0.636277,0.0,0.0,0.0,0.000111,-0.682003,-0.000333,1.41142,-0.728553,-7.605437e-05,0.31129,0.159481,0.115399,-0.636277,0.0,0.0,0.0,0.0,0.0,0.0]
    roll_pitch_pose2=[-0.486326,-1.18821,-0.026531,0.908889,-0.267927,0.130916,0.31129,-0.159481,-0.115399,-0.636277,0.0,0.0,0.0,-0.430362,-0.964194,0.009303,0.590166,-0.173131,0.103544,0.31129,0.159481,0.115399,-0.636277,0.0,0.0,0.0,0.0,0.0,0.0]
    roll_pitch_pose3=[0.463158,0.281851,-0.0701,0.747965,-0.514677,-0.108534,0.31129,-0.159481,-0.115399,-0.636277,0.0,0.0,0.0,0.486068,0.189331,-0.083976,1.08676,-0.76299,-0.139173,0.31129,0.159481,0.115399,-0.636277,0.0,0.0,0.0,0.0,0.0,0.0]
    roll_pitch_poses = [roll_pitch_pose1, roll_pitch_pose2, roll_pitch_pose3]
    hrpsys_version = hcf.seq.ref.get_component_profile().version
    print("hrpsys_version = %s"%hrpsys_version)

def test_kf_plot (test_motion_func, optional_out_file_name): # time [s]
    # Reset KF and store data during motion
    hcf.log_svc.clear()
    test_motion_func()
    hcf.log_svc.save("/tmp/test-kf-samplerobot-{0}".format(optional_out_file_name))
    # Parse data
    #  Estimated base link rpy from KF : time, roll, pitch, yaw
    estimated_base_rpy_ret=[]
    for line in open("/tmp/test-kf-samplerobot-{0}.kf_baseRpyCurrent".format(optional_out_file_name), "r"):
        estimated_base_rpy_ret.append(line.split(" ")[0:-1])
    #  Actual rpy from simualtor : time, posx, posy, posz, roll, pitch, yaw
    act_rpy_ret=[]
    for line in open("/tmp/test-kf-samplerobot-{0}.SampleRobot(Robot)0_WAIST".format(optional_out_file_name), "r"):
        act_rpy_ret.append(line.split(" ")[0:-1])
    #  Get time list
    initial_sec=int(act_rpy_ret[0][0].split(".")[0])
    tm_list=map (lambda x : int(x[0].split(".")[0])-initial_sec + float(x[0].split(".")[1]) * 1e-6, act_rpy_ret)
    # Plotting
    try:
        import matplotlib.pyplot as plt
        plt.clf()
        color_list = ['r', 'g', 'b']
        for idx in range(3):
            plt.plot(tm_list, map(lambda x : 180.0 * float(x[1+3+idx]) / math.pi, act_rpy_ret), color=color_list[idx])
            plt.plot(tm_list, map(lambda x : 180.0 * float(x[1+idx]) / math.pi, estimated_base_rpy_ret), "--", color=color_list[idx])
        plt.xlabel("Time [s]")
        plt.ylabel("Angle [deg]")
        plt.title("KF actual-estimated data (motion time = {0})".format(optional_out_file_name))
        plt.legend(("Actual roll",  "Estimated base roll",
                    "Actual pitch", "Estimated base pitch",
                    "Actual yaw",   "Estimated base yaw"))
        plt.savefig("/tmp/test-kf-samplerobot-data-{0}.eps".format(optional_out_file_name))
    except:
        print >> sys.stderr, "No plot"

def test_bending_common (time, poses):
    hcf.seq_svc.setJointAngles(poses[1], time*0.25)
    hcf.seq_svc.waitInterpolation()
    hcf.seq_svc.setJointAngles(poses[2], time*0.5)
    hcf.seq_svc.waitInterpolation()
    hcf.seq_svc.setJointAngles(poses[0], time*0.25)
    hcf.seq_svc.waitInterpolation()

def test_pitch_bending_4s ():
    test_bending_common(4.0, pitch_poses)

def test_pitch_bending_2s ():
    test_bending_common(2.0, pitch_poses)

def test_roll_bending_4s ():
    test_bending_common(4.0, roll_poses)

def test_roll_bending_2s ():
    test_bending_common(2.0, roll_poses)

def test_yaw_bending_4s ():
    test_bending_common(4.0, yaw_poses)

def test_yaw_bending_2s ():
    test_bending_common(2.0, yaw_poses)

def test_roll_pitch_bending_4s ():
    test_bending_common(4.0, roll_pitch_poses)

def test_roll_pitch_bending_2s ():
    test_bending_common(2.0, roll_pitch_poses)

def test_walk ():
    hcf.abc_svc.goPos(0.1,0,0)
    hcf.abc_svc.waitFootSteps()

def demoGetKalmanFilterParameter():
    print >> sys.stderr, "1. getParameter"
    ret=hcf.kf_svc.getKalmanFilterParam()
    if ret[0]:
        print >> sys.stderr, "  getKalmanFilterParam() => OK"
    assert(ret[0])

def demoSetKalmanFilterParameter():
    print >> sys.stderr, "2. setParameter"
    kfp=hcf.kf_svc.getKalmanFilterParam()[1]
    kfp.Q_angle = 0.001;
    kfp.Q_rate = 0.003;
    kfp.R_angle = 100;
    ret=hcf.kf_svc.setKalmanFilterParam(kfp)
    kfp2=hcf.kf_svc.getKalmanFilterParam()[1]
    ret2 = ret and kfp.Q_angle == kfp2.Q_angle and kfp.Q_rate == kfp2.Q_rate and kfp.R_angle == kfp2.R_angle
    if ret2:
        print >> sys.stderr, "  setKalmanFilterParam() => OK"
    assert(ret2)

def demo():
    init()
    if hrpsys_version >= '315.5.0':
        demoGetKalmanFilterParameter()
        demoSetKalmanFilterParameter()

        # 3. check log and plot
        hcf.abc_svc.startAutoBalancer(["rleg", "lleg"])

        for alg in [OpenHRP.KalmanFilterService.RPYKalmanFilter, OpenHRP.KalmanFilterService.QuaternionExtendedKalmanFilter]:
            kfp=hcf.kf_svc.getKalmanFilterParam()[1]
            kfp.kf_algorithm = alg
            hcf.kf_svc.setKalmanFilterParam(kfp)

            hcf.kf_svc.resetKalmanFilterState()
            hcf.seq_svc.setJointAngles(pitch_poses[0], 1.0)
            hcf.seq_svc.waitInterpolation()
            test_kf_plot(test_pitch_bending_2s, "pitch-bending-2s"+str(alg))

            #hcf.kf_svc.resetKalmanFilterState()
            hcf.seq_svc.setJointAngles(roll_poses[0], 1.0)
            hcf.seq_svc.waitInterpolation()
            test_kf_plot(test_roll_bending_2s, "roll-bending-2s"+str(alg))

            #hcf.kf_svc.resetKalmanFilterState()
            hcf.seq_svc.setJointAngles(yaw_poses[0], 1.0)
            hcf.seq_svc.waitInterpolation()
            test_kf_plot(test_yaw_bending_2s, "yaw-bending-4s"+str(alg))

            #hcf.kf_svc.resetKalmanFilterState()
            hcf.seq_svc.setJointAngles(roll_pitch_poses[0], 1.0)
            hcf.seq_svc.waitInterpolation()
            test_kf_plot(test_roll_pitch_bending_4s, "roll-pitch-bending-4s"+str(alg))

            hcf.seq_svc.setJointAngles(initial_pose, 1.0)
            #hcf.seq_svc.waitInterpolation()
            #hcf.kf_svc.resetKalmanFilterState()
            #hcf.seq_svc.setJointAngles(initial_pose, 1.0)
            hcf.seq_svc.waitInterpolation()
            test_kf_plot(test_walk, "test_walk"+str(alg))

if __name__ == '__main__':
    demo()

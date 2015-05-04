#!/usr/bin/env python

import imp, sys, os, time

# set path to hrpsys to use HrpsysConfigurator
from subprocess import check_output
sys.path.append(os.path.join(check_output(['pkg-config', 'hrpsys-base', '--variable=prefix']).rstrip(),'share/hrpsys/samples/SampleRobot/')) # set path to SampleRobot

import samplerobot_impedance_controller

if __name__ == '__main__':
    samplerobot_impedance_controller.init()
    hcf = samplerobot_impedance_controller.hcf
    initial_pose = [-7.779e-005,  -0.378613,  -0.000209793,  0.832038,  -0.452564,  0.000244781,  0.31129,  -0.159481,  -0.115399,  -0.636277,  0,  0,  0,  -7.77902e-005,  -0.378613,  -0.000209794,  0.832038,  -0.452564,  0.000244781,  0.31129,  0.159481,  0.115399,  -0.636277,  0,  0,  0,  0,  0,  0]
    hcf.seq_svc.setJointAngles(initial_pose, 2.0)
    hcf.seq_svc.waitInterpolation()
    hcf.startImpedance("rarm")
    time.sleep(2)
    hcf.startImpedance("larm")
    time.sleep(2)
    hcf.stopImpedance("larm")
    hcf.stopImpedance("rarm")

## IGNORE ME: this code used for rostest
if [s for s in sys.argv if "--gtest_output=xml:" in s] :
    import unittest, rostest
    rostest.run('hrpsys', 'samplerobot_impedance', unittest.TestCase, sys.argv)







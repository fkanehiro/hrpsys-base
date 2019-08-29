#!/usr/bin/env python

import imp, sys, os, time

# set path to hrpsys to use HrpsysConfigurator
from subprocess import check_output
sys.path.append(os.path.join(check_output(['pkg-config', 'hrpsys-base', '--variable=prefix']).rstrip(),'share/hrpsys/samples/SampleRobot/')) # set path to SampleRobot

import samplerobot_collision_detector
import unittest, rostest

if [s for s in sys.argv if '__name:=samplerobot_co_loop' in s]:
    class TestSampleRobotCollisionDetector(unittest.TestCase):
        def test_demo (self):
            samplerobot_collision_detector.demo_co_loop()
else:
    class TestSampleRobotCollisionDetector(unittest.TestCase):
        def test_demo (self):
            samplerobot_collision_detector.demo()

## IGNORE ME: this code used for rostest
if [s for s in sys.argv if "--gtest_output=xml:" in s] :
    rostest.run('hrpsys', 'samplerobot_collision_detector', TestSampleRobotCollisionDetector, sys.argv)







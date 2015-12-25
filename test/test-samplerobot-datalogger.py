#!/usr/bin/env python

import imp, sys, os, time

# set path to hrpsys to use HrpsysConfigurator
from subprocess import check_output
sys.path.append(os.path.join(check_output(['pkg-config', 'hrpsys-base', '--variable=prefix']).rstrip(),'share/hrpsys/samples/SampleRobot/')) # set path to SampleRobot

import samplerobot_data_logger
import unittest, rostest

class TestSampleRobotDataLogger(unittest.TestCase):
    def test_demo (self):
        samplerobot_data_logger.demo()

## IGNORE ME: this code used for rostest
if [s for s in sys.argv if "--gtest_output=xml:" in s] :
    rostest.run('hrpsys', 'samplerobot_data_logger', TestSampleRobotDataLogger, sys.argv)







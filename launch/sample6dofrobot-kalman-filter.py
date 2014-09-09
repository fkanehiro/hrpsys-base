#!/usr/bin/env python

"""
 this is example file for Sample6dofRobot robot

 $ roslaunch hrpsys sample6dofrobot.launch
 $ rosrun    hrpsys sample6dofrobot-kalman-filter.py

"""

import imp, sys, os

# set path to hrpsys to use HrpsysConfigurator
try:
    imp.find_module('hrpsys') # catkin installed
    sys.path.append(imp.find_module('hrpsys')[1])
except: # rosbuild installed
    import roslib
    roslib.load_manifest('hrpsys')

sys.path.append(os.path.dirname(os.path.abspath(__file__))+'/../share/hrpsys/samples/Sample6dofRobot/') # set path to SampleRobot

import sample6dofrobot_kalman_filter

if __name__ == '__main__':
    sample6dofrobot_kalman_filter.demo()

## IGNORE ME: this code used for rostest
if [s for s in sys.argv if "--gtest_output=xml:" in s] :
    import unittest, rostest
    rostest.run('hrpsys', 'sample6dofrobot_kalman_filter', unittest.TestCase, sys.argv)

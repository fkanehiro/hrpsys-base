#!/usr/bin/env python

"""
 this is example file for SampleRobot robot

 $ roslaunch hrpsys samplerobot.launch CONF_FILE:=`rospack find hrpsys`/samples/SampleRobot/SampleRobot.500.el.conf
 $ rosrun    hrpsys samplerobot-soft-error-limiter.py

"""
import imp, sys, os

# set path to hrpsys to use HrpsysConfigurator
try:
    imp.find_module('hrpsys') # catkin installed
    sys.path.append(imp.find_module('hrpsys')[1])
except: # rosbuild installed
    import roslib
    roslib.load_manifest('hrpsys')

sys.path.append(os.path.dirname(os.path.abspath(__file__))+'/../samples/SampleRobot/') # set path to SampleRobot

import samplerobot_soft_error_limiter

if __name__ == '__main__':
    samplerobot_soft_error_limiter.demo()

## IGNORE ME: this code used for rostest
if [s for s in sys.argv if "--gtest_output=xml:" in s] :
    import unittest, rostest
    rostest.run('hrpsys', 'samplerobot_soft_error_limiter', unittest.TestCase, sys.argv)

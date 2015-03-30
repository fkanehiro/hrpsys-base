#!/usr/bin/env python

"""
 this is example file for SampleRobot robot

 $ roslaunch hrpsys samplerobot.launch CONTROLLER_PERIOD:=200
 $ rosrun    hrpsys samplerobot-stabilizer.py

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

import samplerobot_stabilizer

if __name__ == '__main__':
    samplerobot_stabilizer.demo()

## IGNORE ME: this code used for rostest
if [s for s in sys.argv if "--gtest_output=xml:" in s] :
    import unittest, rostest
    rostest.run('hrpsys', 'samplerobot_stabilizer', unittest.TestCase, sys.argv)







#!/usr/bin/env python

"""
 this is example file for SampleRobot robot terrain walking

 for SlopeUpDown
 $ roslaunch hrpsys samplerobot-terrain-walk.launch MODEL:=SlopeUpDown
 $ rosrun hrpsys samplerobot-terrain-walk.py --SlopeUpDown

 for StairUp
 $ roslaunch hrpsys samplerobot-terrain-walk.launch MODEL:=StairUp
 $ rosrun hrpsys samplerobot-terrain-walk.py --StairUp

 for StairDown
 $ roslaunch hrpsys samplerobot-terrain-walk.launch MODEL:=StairDown
 $ rosrun hrpsys samplerobot-terrain-walk.py --StairDown

"""

import imp, sys, os

from subprocess import check_output
sys.path.append(os.path.join(check_output(['pkg-config', 'hrpsys-base', '--variable=prefix']).rstrip(),'share/hrpsys/samples/SampleRobot/')) # set path to SampleRobot

import samplerobot_terrain_walk

if __name__ == '__main__':
    samplerobot_terrain_walk.demo()
    import sys
    if len(sys.argv) != 2:
        print "Usage:"
        print " ",sys.argv[0]," --StairUp or --StairDown or --SlopeUpDown"
    else:
        if sys.argv[1] == "--StairUp":
            samplerobot_terrain_walk.demoStairUp()
        elif sys.argv[1] == "--StairDown":
            samplerobot_terrain_walk.demoStairDown()
        elif sys.argv[1] == "--SlopeUpDown":
            samplerobot_terrain_walk.demoSlopeUpDown()
        else:
            print "Usage:"
            print " ",sys.argv[0]," --StairUp or --StairDown or --SlopeUpDown"


## IGNORE ME: this code used for rostest
if [s for s in sys.argv if "--gtest_output=xml:" in s] :
    import unittest, rostest
    rostest.run('hrpsys', 'samplerobot_terrain_walk', unittest.TestCase, sys.argv)







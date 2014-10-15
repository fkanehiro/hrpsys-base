#!/usr/bin/env python

PKG = 'hrpsys'
NAME = 'test-samplerobot'

import imp  ## for rosbuild
try:
    imp.find_module(PKG)
except:
    import roslib; roslib.load_manifest(PKG)


import os
import sys
import time
import unittest
import yaml

import rostest
from hrpsys import rtm
from hrpsys.hrpsys_config import *
import OpenHRP

rtm.nsport = 2809

class SampleRobot(HrpsysConfigurator):
    def getRTCList(self):
        return [
            ['seq', "SequencePlayer"],
            ['sh', "StateHolder"],
            ['fk', "ForwardKinematics"],
            ['log', "DataLogger"],
            ]



class TestJointAngle(unittest.TestCase):

    @classmethod
    def setUpClass(self):
        h = SampleRobot()
        h.init(robotname="SampleRobot(Robot)0")

    def test_set_if_find_log(self):
        h = SampleRobot()
        h.findComps()
        print >>sys.stderr, "log=",h.log, "log_svc=",h.log_svc
        self.assertTrue(h.log)
        self.assertTrue(h.log_svc)

    def test_load_pattern(self):
        h = SampleRobot()
        h.findComps()
        from subprocess import check_output
        openhrp3_path = check_output(['rospack','find','openhrp3']).rstrip() # for rosbuild
        PKG_CONFIG_PATH=""
        if os.path.exists(os.path.join(openhrp3_path, "bin")) :
            PKG_CONFIG_PATH='PKG_CONFIG_PATH=%s/lib/pkgconfig:$PKG_CONFIG_PATH'%(openhrp3_path)

        cmd = "%s pkg-config openhrp3.1 --variable=idl_dir"%(PKG_CONFIG_PATH)
        os.path.join(check_output(cmd, shell=True).rstrip(), "../sample/controller/SampleController/etc/Sample")
        h.loadPattern(os.path.join(check_output(cmd, shell=True).rstrip(), "../sample/controller/SampleController/etc/Sample"), 1)
        self.assertEqual(h.waitInterpolation(), None)

    def test_set_joint_angles(self):
        h = SampleRobot()
        h.findComps()
        self.assertTrue(h.setJointAngles(h.getJointAngles(),1))
        self.assertEqual(h.waitInterpolation(), None)

        import random
        a = [(360*random.random()-180) for i in xrange(len(h.getJointAngles()))]
        self.assertTrue(h.setJointAngles(a,2))
        self.assertEqual(h.waitInterpolation(), None)

        a = [0 for i in xrange(len(h.getJointAngles()))]
        self.assertTrue(h.setJointAngles(a,2))
        self.assertEqual(h.waitInterpolation(), None)

#unittest.main()
if __name__ == '__main__':
    rostest.run(PKG, NAME, TestJointAngle, sys.argv)



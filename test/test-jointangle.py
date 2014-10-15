#!/usr/bin/env python

PKG = 'hrpsys'
NAME = 'test-hostname'

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

class PA10(HrpsysConfigurator):
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
        h = PA10()
        h.init(robotname="PA10Controller(Robot)0")

    def test_set_if_find_log(self):
        h = PA10()
        h.findComps()
        print >>sys.stderr, "log=",h.log, "log_svc=",h.log_svc
        self.assertTrue(h.log)
        self.assertTrue(h.log_svc)

    def test_get_joint_angles(self):
        h = PA10()
        h.findComps()
        print >>sys.stderr,  h.getJointAngles()
        self.assertEqual(len(h.getJointAngles()), int(9))

    def test_set_joint_angles(self):
        h = PA10()
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



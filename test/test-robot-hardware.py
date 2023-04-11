#!/usr/bin/env python

PKG = 'hrpsys'
NAME = 'test_robothardware'

from hrpsys import hrpsys_config
from hrpsys import OpenHRP

import socket
from hrpsys import rtm

import unittest
import rostest
import sys

class TestHrpsysRobotHardware(unittest.TestCase):

    def test_rh_service(self):
        try:
            rh = rh_svc = None
            rtm.nshost = 'localhost'
            rtm.nsport = 2809
            rtm.initCORBA()
            rh = rtm.findRTC("RobotHardware0")
            rh_svc = rtm.narrow(rh.service("service0"), "RobotHardwareService")
            print "RTC(RobotHardware0)={0}, {1}".format(rh,rh_svc)
            self.assertTrue(rh and rh_svc)
            rh.start()
            self.assertTrue(rh.isActive())
            self.assertTrue(rh_svc.getStatus())

        except Exception as e:
            print "{0}, RTC(RobotHardware0)={1}, {2}".format(str(e),rh,rh_svc)
            self.fail()
            pass

#unittest.main()
if __name__ == '__main__':
    rostest.run(PKG, NAME, TestHrpsysRobotHardware, sys.argv)

#!/usr/bin/env python

PKG = 'hrpsys'
NAME = 'test-hostname'

import imp  ## for rosbuild
try:
    imp.find_module(PKG)
except:
    import roslib; roslib.load_manifest(PKG)

from hrpsys import hrpsys_config

import socket
import rtm

import unittest
import rostest
import sys
import time

class TestHrpsysHostname(unittest.TestCase):

    def check_initCORBA(self, nshost, nsport=2809):
        try:
            ms = rh = None
            rtm.nshost = nshost
            rtm.nsport = nsport
            rtm.initCORBA()
            count = 0
            while ( not (ms and rh) ) and count < 10:
                ms = rtm.findRTCmanager()
                rh = rtm.findRTC("RobotHardware0")
                if ms and rh :
                    break
                time.sleep(1)
                print >>sys.stderr, "wait for RTCmanager=%r, RTC(RobotHardware0)=%r"%(ms,rh)
                count += 1
            self.assertTrue(ms and rh)
        except Exception as e:
            self.fail("%r, nshost=%r, nsport=%r RTCmanager=%r, RTC(RobotHardware0)=%r"%(str(e),nshost,nsport,ms,rh))
            pass

    def test_gethostname(self):
        self.check_initCORBA(socket.gethostname())
    def test_localhost(self):
        self.check_initCORBA('localhost')
    def test_127_0_0_1(self):
        self.check_initCORBA('127.0.0.1')
    def test_None(self):
        self.check_initCORBA(None)

    @unittest.expectedFailure
    def test_X_unknown(self):
        try:
            self.check_initCORBA('unknown')
        except SystemExit as e:
            print "[This is Expected Failure]"
            print str(e.message)

    @unittest.expectedFailure
    def test_X_123_45_67_89(self):
        try:
            self.check_initCORBA('123.45.67.89')
        except SystemExit as e:
            print "[This is Expected Failure]"
            print str(e.message)

#unittest.main()
if __name__ == '__main__':
    rostest.run(PKG, NAME, TestHrpsysHostname, sys.argv)

#!/usr/bin/env python

PKG = 'hrpsys'
NAME = 'test-hrpsys-config'

import imp  ## for rosbuild
try:
    imp.find_module(PKG)
except:
    import roslib; roslib.load_manifest(PKG)

from hrpsys.hrpsys_config import *
from hrpsys import rtm
import OpenHRP

import argparse,unittest,rostest

import unittest, sys

class SampleHrpsysConfigurator(HrpsysConfigurator):
    def init(self, robotname="SampleRobot(Robot)0", url=""):
         HrpsysConfigurator.init(self, robotname=robotname, url=url)

class TestHrpsysConfig(unittest.TestCase):
    global h

    def test_import_waitinput(self):
        # https://github.com/start-jsk/rtmros_hironx/blob/groovy-devel/hironx_ros_bridge/src/hironx_ros_bridge/hironx_client.py
        from waitInput import waitInputConfirm, waitInputSelect
        self.assertTrue(True)

    def test_findcomp(self):
        global h
        h.findComps()

    def setUp(self):
        global h
        parser = argparse.ArgumentParser(description='hrpsys command line interpreters')
        parser.add_argument('--host', help='corba name server hostname')
        parser.add_argument('--port', help='corba name server port number')
        args, unknown = parser.parse_known_args()

        if args.host:
            rtm.nshost = args.host
        if args.port:
            rtm.nsport = args.port
        h = SampleHrpsysConfigurator()


#unittest.main()
if __name__ == '__main__':
    import rostest
    rostest.run(PKG, NAME, TestHrpsysConfig, sys.argv)

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
    rh = None
    seq = None

    def test_import_waitinput(self):
        # https://github.com/start-jsk/rtmros_hironx/blob/groovy-devel/hironx_ros_bridge/src/hironx_ros_bridge/hironx_client.py
        from waitInput import waitInputConfirm, waitInputSelect
        self.assertTrue(True)

    def test_createcomp(self):
        global h
        self.seq = h.createComp("SequencePlayer",'seq')[0]

    def test_connectcomp(self):
        global h
        if self.seq == None or self.rh == None:
            self.test_createcomp()
        connectPorts(self.rh.port("q"), self.seq.port("qInit"))
        # check number of connection
        assert(len(self.seq.port("qInit").get_connector_profiles()) == 1)
        # check do not connect again if already connected for https://github.com/fkanehiro/hrpsys-base/issues/979
        connectPorts(self.rh.port("q"), self.seq.port("qInit"))
        assert(len(self.seq.port("qInit").get_connector_profiles()) == 1)

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

        h.waitForRTCManager()
        # look for name
        for c in h.ms.get_components():
            if '(Robot)' in c.name() or 'RobotHardware' in c.name():
                h.waitForRobotHardware(c.name())  # get robot hardware name
                break;
        self.rh = h.rh

#unittest.main()
if __name__ == '__main__':
    import rostest
    rostest.run(PKG, NAME, TestHrpsysConfig, sys.argv)

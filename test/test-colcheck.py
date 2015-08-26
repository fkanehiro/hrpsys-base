#!/usr/bin/env python

PKG = 'hrpsys'
NAME = 'test-colcheck'

import sys
print "running test with PYTHONPATH=%s" % ";".join(sys.path)

import imp  ## for rosbuild
try:
    imp.find_module(PKG)
except:
    import roslib; roslib.load_manifest(PKG)

from hrpsys import hrpsys_config

import socket
import rtm

import unittest

class TestHrpsysColcheck(unittest.TestCase):

    def test_dummy(self):
        pass

if __name__ == '__main__':
    try:
        import rostest
        rostest.run(PKG, NAME, TestHrpsysColcheck, sys.argv)
    except ImportError:
        unittest.main()

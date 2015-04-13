#!/usr/bin/env python

try:
    from hrpsys.hrpsys_config import *
    import OpenHRP
except:
    print "import without hrpsys"
    import rtm
    from rtm import *
    from OpenHRP import *
    import waitInput
    from waitInput import *
    import socket
    import time

def getRTCList ():
    return [
        ['seq', "SequencePlayer"],
        ['sh', "StateHolder"],
        ['fk', "ForwardKinematics"]
        ]

def init ():
    global hcf
    hcf = HrpsysConfigurator()
    hcf.getRTCList = getRTCList
    hcf.init ("SampleRobot(Robot)0")

def loadPattern(basename, tm=1.0):
    hcf.loadPattern(basename, tm)
    hcf.waitInterpolation()

def demo():
    init()
    loadPattern("$(OPENHRP_DIR)/share/OpenHRP-3.1/sample/controller/SampleController/etc/Sample")

if __name__ == '__main__':
    demo()

#!/usr/bin/env python

from hrpsys.hrpsys_config import *
from hrpsys import OpenHRP

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
    loadPattern("$(PROJECT_DIR)/../controller/SampleController/etc/Sample")

if __name__ == '__main__':
    demo()

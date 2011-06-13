import rtm

from rtm import *
from OpenHRP import *
from OpenHRP.RobotHardwareServicePackage import *

import socket
import time

def connectComps():
    connectPorts(bridge.port("q"), seq.port("qInit"))
    #
    connectPorts(seq.port("qRef"), hgc.port("qIn"))
    #
    connectPorts(hgc.port("qOut"),  bridge.port("qRef"))
    connectPorts(hgc.port("dqOut"),  bridge.port("dqRef"))
    connectPorts(hgc.port("ddqOut"),  bridge.port("ddqRef"))
    #

def activateComps():
#    rtm.serializeComponents([seq, hgc])
    seq.start()
#    hgc.start()

def createComps():
    global bridge, seq, seq_svc, hgc

    bridge = findRTC("HRP-4C(Robot)0")

    ms.load("SequencePlayer")
    seq = ms.create("SequencePlayer", "seq")
    seq_svc = SequencePlayerServiceHelper.narrow(seq.service("service0"))

    ms.load("HGcontroller")
    hgc = ms.create("HGcontroller", "hgc")

def init():
    global ms

    ms = rtm.findRTCmanager()

    print "creating components"
    createComps()
      
    print "connecting components"
    connectComps()

    print "activating components"
    activateComps()
    print "initialized successfully"

def loadPattern(basename, tm=3.0):
    seq_svc.loadPattern(basename, tm)
    seq_svc.waitInterpolation()

def demo():
    init()
    loadPattern("/home/kanehiro/openrtp/share/hrpsys/samples/HRP-4C/walk2m")

demo()
#init()


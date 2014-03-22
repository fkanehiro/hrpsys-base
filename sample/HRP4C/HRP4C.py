import rtm

from rtm import *
from OpenHRP import *

def connectComps():
    connectPorts(bridge.port("q"), seq.port("qInit"))
    #
    connectPorts(seq.port("qRef"), hgc.port("qIn"))
    #

def activateComps():
    rtm.serializeComponents([bridge, seq])
    seq.start()

def createComps():
    global bridge, seq, seq_svc, hgc

    bridge = findRTC("HRP4C(Robot)0")

    ms.load("SequencePlayer")
    seq = ms.create("SequencePlayer", "seq")
    seq_svc = narrow(seq.service("service0"),"SequencePlayerService")

    hgc = findRTC("HGcontroller0")

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

def loadPattern(basename, tm=1.0):
    seq_svc.loadPattern(basename, tm)
    seq_svc.waitInterpolation()

if __name__ == '__main__':
    initCORBA()
    init()
    loadPattern("data/walk2m")


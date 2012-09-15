import rtm

from rtm import *
from OpenHRP import *

import waitInput
from waitInput import *

import socket
import time

def connectComps():
    connectPorts(rh.port("q"), sh.port("currentQIn"))
    #
    connectPorts(seq.port("qRef"), sh.port("qIn"))
    #
    connectPorts(sh.port("qOut"),  [seq.port("qInit"), qRefPort])
    #

def activateComps():
    rtm.serializeComponents([rh, seq, sh, log])
    rh.start()
    seq.start()
    sh.start()
    log.start()

def createComps():
    global seq, seq_svc, sh, sh_svc, tk_svc, log, log_svc

    ms.load("SequencePlayer")
    seq = ms.create("SequencePlayer", "seq")
    seq_svc = narrow(seq.service("service0"), "SequencePlayerService")

    ms.load("StateHolder")
    sh = ms.create("StateHolder", "StateHolder0")
    sh_svc = narrow(sh.service("service0"), "StateHolderService")
    tk_svc = narrow(sh.service("service1"), "TimeKeeperService")

    ms.load("DataLogger")
    log = ms.create("DataLogger", "log")
    log_svc = narrow(log.service("service0"), "DataLoggerService")
    
def init():
    global ms, rh, rh_svc, ep_svc, simulation_mode, qRefPort

    ms = rtm.findRTCmanager()
    rh = rtm.findRTC("RobotHardware0")
    if rh:
        rh_svc = narrow(rh.service("service0"), "RobotHardwareService")
        ep_svc = narrow(rh.ec, "ExecutionProfileService")
        qRefPort = rh.port("qRef")
    else:
        rh = rtm.findRTC("PA10Controller(Robot)0")
        qRefPort = rtm.findRTC("HGcontroller0").port("qIn")
        simulation_mode = 1
    simulation_mode = 0

    ms = rtm.findRTCmanager()

    print "creating components"
    createComps()
      
    print "connecting components"
    connectComps()

    print "activating components"
    activateComps()
    print "initialized successfully"

def goInitial(tm=3.0):
    seq_svc.setJointAngles([0]*9, tm)

def goActual():
    sh_svc.goActual()

def servoOn(joint="all"):
    rh_svc.servo(joint, SwitchStatus.SWITCH_ON)

def powerOn(joint="all"):
    rh_svc.power(joint, SwitchStatus.SWITCH_ON)

def powerOff(joint="all"):
    rh_svc.power(joint, SwitchStatus.SWITCH_OFF)

def servoOff(joint="all"):
    rh_svc.servo(joint, SwitchStatus.SWITCH_OFF)

def loadPattern(basename, tm=3.0):
    seq_svc.loadPattern(basename, tm)

def setupLogger():
    log_svc.add("TimedDoubleSeq", "pos")
    log_svc.add("TimedLongSeq", "servoState")
    log.owned_ecs[0].start()
    log.start(log.owned_ecs[0])

def demo():
    init()
    setupLogger()
    seq_svc.setJointAngles([0,0.7,0,2.5,0,1.5,0,0,0], 3.0)
    seq_svc.waitInterpolation()
    goInitial(3.0)
    seq_svc.waitInterpolation()
    log_svc.save("/tmp/demo")

demo()

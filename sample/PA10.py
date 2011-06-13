import rtm

from rtm import *
from OpenHRP import *
from OpenHRP.RobotHardwareServicePackage import *

import socket
import time

def connectComps():
    connectPorts(rh.port("q"), sh.port("currentQIn"))
    #
    connectPorts(seq.port("qRef"), sh.port("qIn"))
    #
    connectPorts(sh.port("qOut"),  [seq.port("qInit"), rh.port("qRef")])
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
    seq_svc = SequencePlayerServiceHelper.narrow(seq.service("service0"))

    ms.load("StateHolder")
    sh = ms.create("StateHolder", "StateHolder0")
    sh_svc = StateHolderServiceHelper.narrow(sh.service("service0"))
    tk_svc = TimeKeeperServiceHelper.narrow(sh.service("service1"))

    ms.load("DataLogger")
    log = ms.create("DataLogger", "log")
    log_svc = DataLoggerServiceHelper.narrow(log.service("service0"))
    
def init():
    global ms, rh, rh_svc, servo, ep_svc, simulation_mode

    ms = rtm.findRTCmanager()

    if rtm.findRTC("RobotHardware0") and waitInputSelect("Restart Manager?") == 1:
        ms.restart()

    rh = rtm.findRTC("RobotHardware0")
    rh_svc = RobotHardwareServiceHelper.narrow(rh.service("service0"))
    servo = rh
    ep_svc = ExecutionProfileServiceHelper.narrow(rh.ec)
    simulation_mode = 0

    ms = rtm.findRTCmanager()

    print "creating components"
    createComps()
      
    print "connecting components"
    connectComps()

    print "activating components"
    activateComps()
    print "initialized successfully"

def seqtest():
    seq_svc.loadPattern("/home/hrp2user/etc/test", 1.0)
    seq_svc.waitInterpolation()


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
    connectPorts(rh.port("servoState"),log.port("servoState"))
    log.owned_ecs[0].start()
    log.start(log.owned_ecs[0])

def demo():
    init()
    setupLogger()
    seq_svc.setJointAngles([0.5]*7+[0,0], 3.0)
    seq_svc.waitInterpolation()
    goInitial(3.0)
    seq_svc.waitInterpolation()
    log_svc.save("/tmp/demo")

demo()

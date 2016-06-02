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

def init ():
    global hcf, init_pose, reset_pose, wrench_command0, wrench_command1, hrpsys_version
    hcf = HrpsysConfigurator()
    hcf.getRTCList = hcf.getRTCListUnstable
    hcf.init ("SampleRobot(Robot)0", "$(PROJECT_DIR)/../model/sample1.wrl")
    hrpsys_version = hcf.seq.ref.get_component_profile().version
    print("hrpsys_version = %s"%hrpsys_version)
    if hcf.rfu != None:
        hcf.connectLoggerPort(hcf.rfu, 'ref_rhsensorOut')
        hcf.connectLoggerPort(hcf.rfu, 'ref_lhsensorOut')

# demo functions
def demoReferenceForceUpdater ():
    import numpy as np
    import sys
    i=1;
    print >> sys.stderr, i,". get param";i+=1
    p = hcf.rfu_svc.getReferenceForceUpdaterParam('larm')[1]
    p.update_freq=100
    p.p_gain=0.1
    # set rmfo
    r_fmop = hcf.rmfo_svc.getForceMomentOffsetParam("rhsensor")[1]
    r_fmop.link_offset_centroid = [0,0.0368,-0.076271]
    r_fmop.link_offset_mass = 0.80011
    l_fmop = hcf.rmfo_svc.getForceMomentOffsetParam("lhsensor")[1]
    l_fmop.link_offset_centroid = [0,-0.0368,-0.076271]
    l_fmop.link_offset_mass = 0.80011
    # Set param
    hcf.rmfo_svc.setForceMomentOffsetParam("rhsensor", r_fmop)
    hcf.rmfo_svc.setForceMomentOffsetParam("lhsensor", l_fmop)
    for armName,portName in zip(['rarm', 'larm'],['ref_rhsensorOut','ref_lhsensorOut']):
        hcf.rfu_svc.setReferenceForceUpdaterParam(armName,p)
        print >> sys.stderr, i,". set ref_force from seq [10,0,0]";i+=1
        # set ref_force from seq
        hcf.seq_svc.setWrenches([0]*12+[10,0,0,0,0,0]*2,1);time.sleep(1)
        portData=checkDataPortFromLog(portName)
        print >> sys.stderr, portName,portData[0:3]
        ret = np.linalg.norm(portData) > 9.9;
        assert (ret)
        # start/stop rfu
        print >> sys.stderr, i,". start/stop param for " + armName; i+=1
        ##start rfu
        hcf.rfu_svc.startReferenceForceUpdater(armName);time.sleep(1)
        portData=checkDataPortFromLog(portName)
        print >> sys.stderr, portName,portData[0:3]
        ret = np.linalg.norm(portData) < 0.1;
        assert (ret)
        ##stop rfu
        hcf.rfu_svc.stopReferenceForceUpdater(armName);time.sleep(1)
        portData=checkDataPortFromLog(portName)
        print >> sys.stderr, portName,portData[0:3]
        ret = np.linalg.norm(portData) > 9.9;
        assert (ret)
        # reset ref_force from seq
        print >> sys.stderr, i,". set ref_force from seq [0,0,0]";i+=1
        hcf.seq_svc.setWrenches([0]*24,1);time.sleep(1)

def saveLogForCheckParameter(log_fname="/tmp/test-samplerobot-reference-force-updater-check-port"):
    hcf.setMaxLogLength(1);hcf.clearLog();time.sleep(0.1);hcf.saveLog(log_fname)

def checkDataPortFromLog(port_name, log_fname="/tmp/test-samplerobot-reference-force-updater-check-port",save_log=True, rtc_name="rfu"):
    if save_log:
        saveLogForCheckParameter(log_fname)
    return map(float, open(log_fname+"."+rtc_name+"_"+port_name, "r").readline().split(" ")[1:-1])

def demo():
    init()
    if hrpsys_version >= '315.9.0':
        demoReferenceForceUpdater()

if __name__ == '__main__':
    demo()

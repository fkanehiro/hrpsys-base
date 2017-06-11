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

import sys

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
def demoGetReferecenForceUpdateParam ():
    print >> sys.stderr, "1. getParam"
    for limb_name in ['rarm', 'larm']:
        [ret, rfup] = hcf.rfu_svc.getReferenceForceUpdaterParam(limb_name)
        assert(ret)
    [ret, rfup] = hcf.rfu_svc.getReferenceForceUpdaterParam('rarm2') # Invalid name
    assert(not ret)
    print >> sys.stderr, "  =>OK"

def demoSetReferecenForceUpdateParam ():
    print >> sys.stderr, "2. setParam"
    for limb_name in ['rarm', 'larm']:
        [ret, rfup] = hcf.rfu_svc.getReferenceForceUpdaterParam(limb_name)
        ret = hcf.rfu_svc.setReferenceForceUpdaterParam(limb_name, rfup)
        assert(ret)
    ret = hcf.rfu_svc.setReferenceForceUpdaterParam('rarm2', rfup) # Invalid name
    assert(not ret)
    print >> sys.stderr, "  =>OK"

def demoReferenceForceUpdater ():
    print >> sys.stderr, "3. Reference Force Update"
    import numpy as np
    i=1;
    #print >> sys.stderr, i,". get param";i+=1
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
        print >> sys.stderr, " 3."+str(i)+". set ref_force from seq [10,0,0]";i+=1
        # set ref_force from seq
        hcf.seq_svc.setWrenches([0]*12+[10,0,0,0,0,0]*2,1);time.sleep(1)
        portData=checkDataPortFromLog(portName)
        print >> sys.stderr, portName,portData[0:3]
        ret = np.linalg.norm(portData) > 9.9;
        assert (ret)
        # start/stop rfu
        print >> sys.stderr, " 3."+str(i)+". start/stop param for " + armName; i+=1
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
        print >> sys.stderr, " 3."+str(i)+". set ref_force from seq [0,0,0]";i+=1
        hcf.seq_svc.setWrenches([0]*24,1);time.sleep(1)

def demoSetReferecenForceUpdateParamWhileActive ():
    print >> sys.stderr, "4. setParam while active"
    print >> sys.stderr, " 4.0 Start"
    hcf.rfu_svc.startReferenceForceUpdater('rarm')
    print >> sys.stderr, " 4.1 Check setParam without any changes"
    [ret, rfup_org] = hcf.rfu_svc.getReferenceForceUpdaterParam('rarm')
    ret = hcf.rfu_svc.setReferenceForceUpdaterParam('rarm', rfup_org)
    assert(ret)
    print >> sys.stderr, " 4.2 Check setParam which cannot be changed while active"
    [ret, rfup] = hcf.rfu_svc.getReferenceForceUpdaterParam('rarm')
    rfup.frame = 'world'
    ret = hcf.rfu_svc.setReferenceForceUpdaterParam('rarm', rfup)
    assert(not ret)
    [ret, rfup] = hcf.rfu_svc.getReferenceForceUpdaterParam('rarm')
    rfup.update_freq = rfup.update_freq*10
    ret = hcf.rfu_svc.setReferenceForceUpdaterParam('rarm', rfup)
    assert(not ret)
    [ret, rfup] = hcf.rfu_svc.getReferenceForceUpdaterParam('rarm')
    rfup.update_time_ratio = rfup.update_time_ratio*10
    ret = hcf.rfu_svc.setReferenceForceUpdaterParam('rarm', rfup)
    assert(not ret)
    print >> sys.stderr, " 4.3 Check setParam which can be changed while active"
    [ret, rfup] = hcf.rfu_svc.getReferenceForceUpdaterParam('rarm')
    rfup.motion_dir=[0,0,-1]
    ret = hcf.rfu_svc.setReferenceForceUpdaterParam('rarm', rfup)
    assert(ret)
    rfup.p_gain = rfup.p_gain*0.1
    ret = hcf.rfu_svc.setReferenceForceUpdaterParam('rarm', rfup)
    assert(ret)
    rfup.d_gain = rfup.d_gain*0.1
    ret = hcf.rfu_svc.setReferenceForceUpdaterParam('rarm', rfup)
    assert(ret)
    rfup.i_gain = rfup.i_gain*0.1
    ret = hcf.rfu_svc.setReferenceForceUpdaterParam('rarm', rfup)
    assert(ret)
    rfup.is_hold_value = not rfup.is_hold_value
    ret = hcf.rfu_svc.setReferenceForceUpdaterParam('rarm', rfup)
    assert(ret)
    print >> sys.stderr, " 4.4 Stop"
    hcf.rfu_svc.stopReferenceForceUpdater('rarm')
    hcf.rfu_svc.setReferenceForceUpdaterParam('rarm', rfup_org)
    print >> sys.stderr, "  =>OK"

def demoReferecenForceUpdateParamFootOriginExtMoment ():
    print >> sys.stderr, "5. FootOriginExtMoment"
    ret = hcf.rfu_svc.startReferenceForceUpdater('footoriginextmoment')
    ret = hcf.rfu_svc.stopReferenceForceUpdater('footoriginextmoment') and ret
    assert(ret)
    print >> sys.stderr, "  =>OK"

def saveLogForCheckParameter(log_fname="/tmp/test-samplerobot-reference-force-updater-check-port"):
    hcf.setMaxLogLength(1);hcf.clearLog();time.sleep(0.1);hcf.saveLog(log_fname)

def checkDataPortFromLog(port_name, log_fname="/tmp/test-samplerobot-reference-force-updater-check-port",save_log=True, rtc_name="rfu"):
    if save_log:
        saveLogForCheckParameter(log_fname)
    return map(float, open(log_fname+"."+rtc_name+"_"+port_name, "r").readline().split(" ")[1:-1])

def demo():
    init()
    from distutils.version import StrictVersion
    if StrictVersion(hrpsys_version) >= StrictVersion('315.9.0'):
        demoGetReferecenForceUpdateParam()
        demoSetReferecenForceUpdateParam()
        demoReferenceForceUpdater()
        if StrictVersion(hrpsys_version) >= StrictVersion('315.13.0'):
            demoSetReferecenForceUpdateParamWhileActive()
            demoReferecenForceUpdateParamFootOriginExtMoment()

if __name__ == '__main__':
    demo()

#!/usr/bin/env python

try:
    from hrpsys.hrpsys_config import *
    import OpenHRP
except:
    print("import without hrpsys")
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
    print(("hrpsys_version = %s"%hrpsys_version))
    if hcf.rfu != None:
        hcf.connectLoggerPort(hcf.rfu, 'ref_rhsensorOut')
        hcf.connectLoggerPort(hcf.rfu, 'ref_lhsensorOut')

# demo functions
def demoGetReferecenForceUpdateParam ():
    print("1. getParam", file=sys.stderr)
    for limb_name in ['rarm', 'larm']:
        [ret, rfup] = hcf.rfu_svc.getReferenceForceUpdaterParam(limb_name)
        assert(ret)
    [ret, rfup] = hcf.rfu_svc.getReferenceForceUpdaterParam('rarm2') # Invalid name
    assert(not ret)
    print("  =>OK", file=sys.stderr)

def demoSetReferecenForceUpdateParam ():
    print("2. setParam", file=sys.stderr)
    print("  Valid limb access", file=sys.stderr)
    for limb_name in ['rarm', 'larm']:
        [ret, rfup] = hcf.rfu_svc.getReferenceForceUpdaterParam(limb_name)
        ret = hcf.rfu_svc.setReferenceForceUpdaterParam(limb_name, rfup)
        assert(ret)
    print("  Invalid limb access", file=sys.stderr)
    ret = hcf.rfu_svc.setReferenceForceUpdaterParam('rarm2', rfup) # Invalid name
    assert(not ret)
    print("  =>OK", file=sys.stderr)

def demoReferenceForceUpdater ():
    print("3. Reference Force Update", file=sys.stderr)
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
        print(" 3."+str(i)+". set ref_force from seq [10,0,0]", file=sys.stderr);i+=1
        # set ref_force from seq
        hcf.seq_svc.setWrenches([0]*12+[10,0,0,0,0,0]*2,1);time.sleep(1)
        portData=checkDataPortFromLog(portName)
        print(portName,portData[0:3], file=sys.stderr)
        ret = np.linalg.norm(portData) > 9.9;
        assert (ret)
        # start/stop rfu
        print(" 3."+str(i)+". start/stop param for " + armName, file=sys.stderr); i+=1
        ##start rfu
        hcf.rfu_svc.startReferenceForceUpdater(armName);time.sleep(1)
        portData=checkDataPortFromLog(portName)
        print(portName,portData[0:3], file=sys.stderr)
        ret = np.linalg.norm(portData) < 0.1;
        assert (ret)
        ##stop rfu
        hcf.rfu_svc.stopReferenceForceUpdater(armName);time.sleep(1)
        portData=checkDataPortFromLog(portName)
        print(portName,portData[0:3], file=sys.stderr)
        ret = np.linalg.norm(portData) > 9.9;
        assert (ret)
        # reset ref_force from seq
        print(" 3."+str(i)+". set ref_force from seq [0,0,0]", file=sys.stderr);i+=1
        hcf.seq_svc.setWrenches([0]*24,1);time.sleep(1)

def demoSetReferecenForceUpdateParamWhileActive ():
    print("4. setParam while active", file=sys.stderr)
    print(" 4.0 Start", file=sys.stderr)
    hcf.rfu_svc.startReferenceForceUpdater('rarm')
    print(" 4.1 Check setParam without any changes", file=sys.stderr)
    [ret, rfup_org] = hcf.rfu_svc.getReferenceForceUpdaterParam('rarm')
    ret = hcf.rfu_svc.setReferenceForceUpdaterParam('rarm', rfup_org)
    assert(ret)
    print(" 4.2 Check setParam which cannot be changed while active", file=sys.stderr)
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
    print(" 4.3 Check setParam which can be changed while active", file=sys.stderr)
    [ret, rfup] = hcf.rfu_svc.getReferenceForceUpdaterParam('rarm')
    rfup.motion_dir = tmp_value = [0,0,-1]
    ret = hcf.rfu_svc.setReferenceForceUpdaterParam('rarm', rfup)
    print("   motion_dir ...", file=sys.stderr)
    assert(ret and (list(map (lambda x,y : abs(x-y)<1e-5, hcf.rfu_svc.getReferenceForceUpdaterParam('rarm')[1].motion_dir, tmp_value))))
    rfup.p_gain = tmp_value = rfup.p_gain*0.1
    ret = hcf.rfu_svc.setReferenceForceUpdaterParam('rarm', rfup)
    print("   p_gain ...", file=sys.stderr)
    assert(ret and (abs(hcf.rfu_svc.getReferenceForceUpdaterParam('rarm')[1].p_gain-tmp_value) < 1e-5))
    rfup.d_gain = tmp_value = rfup.d_gain*0.1
    ret = hcf.rfu_svc.setReferenceForceUpdaterParam('rarm', rfup)
    print("   d_gain ...", file=sys.stderr)
    assert(ret and (abs(hcf.rfu_svc.getReferenceForceUpdaterParam('rarm')[1].d_gain-tmp_value) < 1e-5))
    rfup.i_gain = rfup.i_gain*0.1
    ret = hcf.rfu_svc.setReferenceForceUpdaterParam('rarm', rfup)
    print("   i_gain ...", file=sys.stderr)
    assert(ret and (abs(hcf.rfu_svc.getReferenceForceUpdaterParam('rarm')[1].i_gain-tmp_value) < 1e-5))
    rfup.is_hold_value = tmp_value = not rfup.is_hold_value
    ret = hcf.rfu_svc.setReferenceForceUpdaterParam('rarm', rfup)
    print("   is_hold_value ...", file=sys.stderr)
    assert(ret and hcf.rfu_svc.getReferenceForceUpdaterParam('rarm')[1].is_hold_value == tmp_value)
    print(" 4.4 Stop", file=sys.stderr)
    hcf.rfu_svc.stopReferenceForceUpdater('rarm')
    hcf.rfu_svc.setReferenceForceUpdaterParam('rarm', rfup_org)
    print("  =>OK", file=sys.stderr)

def demoReferecenForceUpdateParamFootOriginExtMoment ():
    print("5. FootOriginExtMoment", file=sys.stderr)
    ret = hcf.rfu_svc.startReferenceForceUpdater('footoriginextmoment')
    ret = hcf.rfu_svc.stopReferenceForceUpdater('footoriginextmoment') and ret
    assert(ret)
    print("  =>OK", file=sys.stderr)

def saveLogForCheckParameter(log_fname="/tmp/test-samplerobot-reference-force-updater-check-port"):
    hcf.setMaxLogLength(1);hcf.clearLog();time.sleep(0.1);hcf.saveLog(log_fname)

def checkDataPortFromLog(port_name, log_fname="/tmp/test-samplerobot-reference-force-updater-check-port",save_log=True, rtc_name="rfu"):
    if save_log:
        saveLogForCheckParameter(log_fname)
    return list(map(float, open(log_fname+"."+rtc_name+"_"+port_name, "r").readline().split(" ")[1:-1]))

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

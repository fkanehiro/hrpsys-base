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
    global hcf, initial_pose, hrpsys_version
    hcf = HrpsysConfigurator()
    hcf.getRTCList = hcf.getRTCListUnstable
    hcf.init ("SampleRobot(Robot)0", "$(PROJECT_DIR)/../model/sample1.wrl")
    # set initial pose from sample/controller/SampleController/etc/Sample.pos
    initial_pose = [-7.779e-005,  -0.378613,  -0.000209793,  0.832038,  -0.452564,  0.000244781,  0.31129,  -0.159481,  -0.115399,  -0.636277,  0,  0,  0.637045,  -7.77902e-005,  -0.378613,  -0.000209794,  0.832038,  -0.452564,  0.000244781,  0.31129,  0.159481,  0.115399,  -0.636277,  0,  0,  -0.637045,  0,  0,  0]
    hcf.seq_svc.setJointAngles(initial_pose, 2.5)
    hcf.waitInterpolation()
    hrpsys_version = hcf.seq.ref.get_component_profile().version
    print("hrpsys_version = %s"%hrpsys_version)

def saveLogForCheckParameter(log_fname="/tmp/test-samplerobot-remove-force-offset-check-param"):
    hcf.setMaxLogLength(1);hcf.clearLog();time.sleep(0.1);hcf.saveLog(log_fname)

def checkParameterFromLog(port_name, log_fname="/tmp/test-samplerobot-remove-force-offset-check-param", save_log=True, rtc_name="rmfo"):
    if save_log:
        saveLogForCheckParameter(log_fname)
    return map(float, open(log_fname+"."+rtc_name+"_"+port_name, "r").readline().split(" ")[1:-1])

def demoGetForceMomentOffsetParam ():
    print >> sys.stderr, "1. GetForceMomentOffsetParam"
    for fs_name in ["rhsensor", "lhsensor"]:
        ret = hcf.rmfo_svc.getForceMomentOffsetParam(fs_name)
        if ret[0]:
            print >> sys.stderr, "    getForceMomentOffsetParam('", fs_name,"') => OK"
        assert(ret[0] is True)

def demoSetForceMomentOffsetParam ():
    print >> sys.stderr, "2. SetForceMomentOffsetParam"
    print >> sys.stderr, "  Force and moment are large because of link offsets"
    saveLogForCheckParameter()
    for fs_name in ["rhsensor", "lhsensor"]:
        fm = numpy.linalg.norm(checkParameterFromLog("off_"+fs_name, save_log=False))
        vret = fm > 5e-2
        print >> sys.stderr, "    no-offset-removed force moment (",fs_name,") ", fm, "=> ", vret
        assert(vret)
    print >> sys.stderr, "  Set link offsets (link_offset_centroid and link_offset_mass are identified value)."
    # Get param
    r_fmop = hcf.rmfo_svc.getForceMomentOffsetParam("rhsensor")[1]
    r_fmop.link_offset_centroid = [0,0.0368,-0.076271]
    r_fmop.link_offset_mass = 0.80011
    l_fmop = hcf.rmfo_svc.getForceMomentOffsetParam("lhsensor")[1]
    l_fmop.link_offset_centroid = [0,-0.0368,-0.076271]
    l_fmop.link_offset_mass = 0.80011
    # Set param
    hcf.rmfo_svc.setForceMomentOffsetParam("rhsensor", r_fmop)
    hcf.rmfo_svc.setForceMomentOffsetParam("lhsensor", l_fmop)
    # Check values
    ret = hcf.rmfo_svc.getForceMomentOffsetParam("rhsensor")
    if ret[0] and ret[1].link_offset_mass == r_fmop.link_offset_mass and ret[1].link_offset_centroid == r_fmop.link_offset_centroid:
        print >> sys.stderr, "    getForceMomentOffsetParam('rhsensor') => OK"
    assert((ret[0] and ret[1].link_offset_mass == r_fmop.link_offset_mass and ret[1].link_offset_centroid == r_fmop.link_offset_centroid))
    ret = hcf.rmfo_svc.getForceMomentOffsetParam("lhsensor")
    if ret[0] and ret[1].link_offset_mass == l_fmop.link_offset_mass and ret[1].link_offset_centroid == l_fmop.link_offset_centroid:
        print >> sys.stderr, "    getForceMomentOffsetParam('lhsensor') => OK"
    assert((ret[0] and ret[1].link_offset_mass == l_fmop.link_offset_mass and ret[1].link_offset_centroid == l_fmop.link_offset_centroid))
    print >> sys.stderr, "  Force and moment are reduced"
    saveLogForCheckParameter()
    for fs_name in ["rhsensor", "lhsensor"]:
        fm = numpy.linalg.norm(checkParameterFromLog("off_"+fs_name, save_log=False))
        vret = fm < 5e-2
        print >> sys.stderr, "    no-offset-removed force moment (",fs_name,") ", fm, "=> ", vret
        assert(vret)

def demoDumpLoadForceMomentOffsetParams():
    print >> sys.stderr, "3. Dump and load parameter file"
    print >> sys.stderr, "  Get and set param"
    r_fmop = hcf.rmfo_svc.getForceMomentOffsetParam("rhsensor")[1]
    r_fmop.link_offset_centroid = [0,0.0368,-0.076271]
    r_fmop.link_offset_mass = 0.80011
    l_fmop = hcf.rmfo_svc.getForceMomentOffsetParam("lhsensor")[1]
    l_fmop.link_offset_centroid = [0,-0.0368,-0.076271]
    l_fmop.link_offset_mass = 0.80011
    hcf.rmfo_svc.setForceMomentOffsetParam("rhsensor", r_fmop)
    hcf.rmfo_svc.setForceMomentOffsetParam("lhsensor", l_fmop)
    print >> sys.stderr, "  Dump param as file"
    ret = hcf.rmfo_svc.dumpForceMomentOffsetParams("/tmp/test-rmfo-offsets.dat")
    print >> sys.stderr, "  Value check"
    data_str=filter(lambda x : x.find("lhsensor") >= 0, open("/tmp/test-rmfo-offsets.dat", "r").read().split("\n"))[0]
    vcheck = map(float, data_str.split(" ")[7:10]) == l_fmop.link_offset_centroid and float(data_str.split(" ")[10]) == l_fmop.link_offset_mass
    data_str=filter(lambda x : x.find("rhsensor") >= 0, open("/tmp/test-rmfo-offsets.dat", "r").read().split("\n"))[0]
    vcheck = vcheck and map(float, data_str.split(" ")[7:10]) == r_fmop.link_offset_centroid and float(data_str.split(" ")[10]) == r_fmop.link_offset_mass
    import os
    if ret and os.path.exists("/tmp/test-rmfo-offsets.dat") and vcheck:
        print >> sys.stderr, "    dumpForceMomentOffsetParams => OK"
    assert((ret and os.path.exists("/tmp/test-rmfo-offsets.dat") and vcheck))
    print >> sys.stderr, "  Resetting values"
    r_fmop2 = hcf.rmfo_svc.getForceMomentOffsetParam("rhsensor")[1]
    r_fmop2.link_offset_centroid = [0,0,0]
    r_fmop2.link_offset_mass = 0
    l_fmop2 = hcf.rmfo_svc.getForceMomentOffsetParam("lhsensor")[1]
    l_fmop2.link_offset_centroid = [0,0,0]
    l_fmop2.link_offset_mass = 0
    hcf.rmfo_svc.setForceMomentOffsetParam("rhsensor", r_fmop2)
    hcf.rmfo_svc.setForceMomentOffsetParam("lhsensor", l_fmop2)
    print >> sys.stderr, "  Load from file"
    ret = hcf.rmfo_svc.loadForceMomentOffsetParams("/tmp/test-rmfo-offsets.dat")
    r_fmop3 = hcf.rmfo_svc.getForceMomentOffsetParam("rhsensor")[1]
    l_fmop3 = hcf.rmfo_svc.getForceMomentOffsetParam("lhsensor")[1]
    vcheck = r_fmop3.link_offset_mass == r_fmop.link_offset_mass and r_fmop3.link_offset_centroid == r_fmop.link_offset_centroid and l_fmop3.link_offset_mass == l_fmop.link_offset_mass and l_fmop3.link_offset_centroid == l_fmop.link_offset_centroid
    if ret and vcheck:
        print >> sys.stderr, "    loadForceMomentOffsetParams => OK"
    assert((ret and vcheck))

def demo():
    import numpy
    init()
    if hrpsys_version >= '315.5.0':
        demoGetForceMomentOffsetParam()
        demoSetForceMomentOffsetParam()
        demoDumpLoadForceMomentOffsetParams()

if __name__ == '__main__':
    demo()

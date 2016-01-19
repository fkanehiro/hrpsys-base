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
    global hcf, hrpsys_version
    hcf = HrpsysConfigurator()
    hcf.init ("SampleRobot(Robot)0", "$(PROJECT_DIR)/../model/sample1.wrl")
    hcf.connectLoggerPort(hcf.sh, 'optionalDataOut') # Just for checking
    global reset_pose_doc, move_base_pose_doc, doc
    # doc for patterns.
    #  torque and wrenches are non-realistic values, just for testing.
    dof = 29
    reset_pose_doc = {'pos':[-7.778932e-05,-0.378613,-0.00021,0.832039,-0.452564,0.000245,0.31129,-0.159481,-0.115399,-0.636277,0.0,0.0,0.0,-7.778932e-05,-0.378613,-0.00021,0.832039,-0.452564,0.000245,0.31129,0.159481,0.115399,-0.636277,0.0,0.0,0.0,0.0,0.0,0.0],
                      'vel':[0]*dof,
                      'zmp':[-0.00081, 1.712907e-05, -0.66815],
#                      'gsens':[0,0,0],
                      'waist':[0.000234, 0.000146, 0.66815, -0.000245, -0.000862, 0.000195],
                      'waist_acc':[0]*3,
                      'torque':[0]*dof, # non realistic value
                      'wrenches':[0]*24, # non realistic value
                      'optionaldata':[1,1,0,0,1,1,1,1]
                      }
    move_base_pose_doc = {'pos':[8.251963e-05,-0.980029,-0.000384,1.02994,-0.398115,-0.000111,0.31129,-0.159481,-0.115399,-0.636277,0.0,0.0,0.0,8.252625e-05,-0.980033,-0.000384,1.02986,-0.398027,-0.000111,0.31129,0.159481,0.115399,-0.636277,0.0,0.0,0.0,0.0,0.0,0.0],
                          'vel':[0]*dof,
                          'zmp':[0.302518, 0.000153, -0.562325],
#                          'gsens':[0,0,0],
                          'waist':[-0.092492, -6.260780e-05, 0.6318, -0.000205, 0.348204, 0.000268],
                          'waist_acc':[0]*3,
                          'torque':range(dof), # non realistic value
                          'wrenches':[1]*6+[-2]*6+[3]*6+[-4]*6, # non realistic value
                          'optionaldata':[0,1,0,0,0.1,0.1,0.1,0.1] # non realistic value
                          }
    hrpsys_version = hcf.seq.ref.get_component_profile().version
    print("hrpsys_version = %s"%hrpsys_version)
    hcf.seq_svc.removeJointGroup('larm')
    hcf.seq_svc.setJointAngles(reset_pose_doc['pos'], 1.0);
    hcf.seq_svc.waitInterpolation();


def dumpLoadPatternTestFile (basename, var_doc, tm):
    for key in var_doc.keys():
        f=open(basename+"."+key, "w")
        dumpstr=" ".join(map(str, [tm]+var_doc[key]))
        f.writelines(dumpstr)
        f.close()

def checkArrayEquality (arr1, arr2, eps=1e-7):
    return all(map(lambda x,y : abs(x-y)<eps, arr1, arr2))

def checkArrayBetween (arr1, arr2, arr3, eps=1e-7):
    return all(map(lambda x,y,z : (z-y)*(x-y) <= eps, arr1, arr2, arr3))

def saveLogForCheckParameter(log_fname="/tmp/test-samplerobot-sequence-player-check-param"):
    hcf.setMaxLogLength(1);hcf.clearLog();time.sleep(0.1);hcf.saveLog(log_fname)

def checkParameterFromLog(port_name, log_fname="/tmp/test-samplerobot-sequence-player-check-param", save_log=True, rtc_name="sh"):
    if save_log:
        saveLogForCheckParameter(log_fname)
    return map(float, open(log_fname+"."+rtc_name+"_"+port_name, "r").readline().split(" ")[1:-1])

def checkJointAngles (var_doc):
    if isinstance(var_doc, list):
        p = var_doc
    else:
        p = var_doc['pos']
    ret = checkArrayEquality(hcf.sh_svc.getCommand().jointRefs, p)
    print "  pos => ", ret

def checkJointAnglesBetween(from_doc, to_doc):
    p0 =  from_doc if isinstance(from_doc, list) else from_doc['pos']
    p1 =    to_doc if isinstance(  to_doc, list) else   to_doc['pos']
    ret = checkArrayBetween(p0, hcf.sh_svc.getCommand().jointRefs, p1)
    print "  pos => ", ret
    assert(ret is True)

def checkZmp(var_doc):
    zmp=hcf.sh_svc.getCommand().zmp
    ret = checkArrayEquality([zmp[0], zmp[1], zmp[2]], var_doc['zmp'])
    print "  zmp => ", ret
    assert(ret is True)

def checkWaist(var_doc, save_log=True):
    bpos=checkParameterFromLog("basePosOut", save_log=save_log)
    brpy=checkParameterFromLog("baseRpyOut", save_log=False)
    ret = checkArrayEquality([bpos[0], bpos[1], bpos[2], brpy[0], brpy[1], brpy[2]], var_doc['waist'], eps=1e-5)
    print "  waist => ", ret
    assert(ret is True)

def checkTorque (var_doc, save_log=True):
    ret = checkArrayEquality(checkParameterFromLog("tqOut", save_log=save_log), var_doc['torque'], eps=1e-5)
    print "  torque => ", ret
    assert(ret is True)

def checkWrenches (var_doc, save_log=True):
    if save_log:
        saveLogForCheckParameter()
    ret = checkArrayEquality(reduce(lambda x,y:x+y, map(lambda fs : checkParameterFromLog(fs+"Out", save_log=False), ['lfsensor', 'rfsensor', 'lhsensor', 'rhsensor'])), var_doc['wrenches'], eps=1e-5)
    print "  wrenches => ", ret
    assert(ret is True)

def checkOptionalData (var_doc, save_log=True):
    ret = checkArrayEquality(checkParameterFromLog("optionalDataOut", save_log=save_log), var_doc['optionaldata'], eps=1e-5)
    print "  optionaldata => ", ret
    assert(ret is True)

def checkRobotState (var_doc):
    checkJointAngles(var_doc)
    checkZmp(var_doc)
    checkWaist(var_doc)
    checkTorque(var_doc, save_log=False)
    if hrpsys_version >= '315.2.0':
        checkWrenches(var_doc, save_log=False)
        checkOptionalData(var_doc, save_log=False)

# demo functions
def demoSetJointAngles():
    print >> sys.stderr, "1. setJointAngles"
    hcf.seq_svc.setJointAngles(move_base_pose_doc['pos'], 1.0);
    hcf.seq_svc.waitInterpolation();
    checkJointAngles(move_base_pose_doc)
    hcf.seq_svc.setJointAngles(reset_pose_doc['pos'], 1.0);
    hcf.seq_svc.waitInterpolation();
    checkJointAngles(reset_pose_doc)
    # check override
    print >> sys.stderr, "   check override"
    hcf.seq_svc.setJointAngles(move_base_pose_doc['pos'], 5.0);
    time.sleep(2.5)
    hcf.seq_svc.setJointAngles(reset_pose_doc['pos'], 1.0);
    hcf.seq_svc.waitInterpolation();
    checkJointAngles(reset_pose_doc)
    # check clear
    if hrpsys_version < '315.5.0':
        return
    print >> sys.stderr, "   check clear"
    hcf.seq_svc.setJointAngles(move_base_pose_doc['pos'], 5.0);
    time.sleep(2.5)
    hcf.seq_svc.clearJointAngles()
    checkJointAnglesBetween(reset_pose_doc,move_base_pose_doc)

def demoSetJointAnglesSequence():
    print >> sys.stderr, "2. setJointAnglesSequence"
    hcf.seq_svc.setJointAnglesSequence([move_base_pose_doc['pos'],reset_pose_doc['pos'],move_base_pose_doc['pos']], [1.0,1.0,1.0]);
    hcf.seq_svc.waitInterpolation();
    checkJointAngles(move_base_pose_doc)
    hcf.seq_svc.setJointAnglesSequence([reset_pose_doc['pos']], [1.0]);
    hcf.seq_svc.waitInterpolation();
    checkJointAngles(reset_pose_doc)
    # check override
    print >> sys.stderr, "   check override"
    hcf.seq_svc.setJointAnglesSequence([move_base_pose_doc['pos'],reset_pose_doc['pos'],move_base_pose_doc['pos']], [1.0,1.0,5.0])
    time.sleep(3.5)
    hcf.seq_svc.setJointAnglesSequence([reset_pose_doc['pos'],move_base_pose_doc['pos'],reset_pose_doc['pos']], [1.0,1.0,1.0]);
    hcf.seq_svc.waitInterpolation();
    checkJointAngles(reset_pose_doc)
    # check clear
    print >> sys.stderr, "   check clear"
    hcf.seq_svc.setJointAnglesSequence([move_base_pose_doc['pos'],reset_pose_doc['pos'],move_base_pose_doc['pos']], [1.0,1.0,5.0])
    time.sleep(3.5)
    hcf.seq_svc.clearJointAngles()
    checkJointAnglesBetween(reset_pose_doc,move_base_pose_doc)

def demoSetJointAngle():
    print >> sys.stderr, "3. setJointAngle"
    hcf.seq_svc.setJointAngles(reset_pose_doc['pos'], 1.0);
    hcf.seq_svc.waitInterpolation();
    hcf.seq_svc.setJointAngle("WAIST_R", 10*3.14159/180.0, 1.0);
    hcf.seq_svc.waitInterpolation();
    p = list(reset_pose_doc['pos']) # copy object
    p[27] = 10*3.14159/180.0
    checkJointAngles(p)
    hcf.seq_svc.setJointAngle("WAIST_R", 0*3.14159/180.0, 1.0);
    hcf.seq_svc.waitInterpolation();
    checkJointAngles(reset_pose_doc)
    # # check override
    # print "   check override"
    # hcf.seq_svc.setJointAngles(reset_pose_doc['pos'], 1.0);
    # hcf.seq_svc.waitInterpolation();
    # hcf.seq_svc.setJointAngle("WAIST_R", 10*3.14159/180.0, 5.0);
    # time.sleep(2.5)
    # hcf.seq_svc.setJointAngle("WAIST_R", 0*3.14159/180.0, 5.0);
    # hcf.seq_svc.waitInterpolation();
    # checkJointAngles(reset_pose_doc)
    # # check clear
    # print "   check clear"
    # hcf.seq_svc.setJointAngle("WAIST_R", 10*3.14159/180.0, 5.0);
    # time.sleep(2.5)
    # hcf.seq_svc.clearJointAngles()
    # checkJointAnglesBetween(reset_pose_doc,p)

def demoLoadPattern():
    print >> sys.stderr, "4. loadPattern"
    # dump pattern doc as loadPattern file
    dumpLoadPatternTestFile("/tmp/test-samplerobot-move-base-pose", move_base_pose_doc, 2.0);
    dumpLoadPatternTestFile("/tmp/test-samplerobot-reset-pose", reset_pose_doc, 2.0);
    # execute loadPattern and check final values
    hcf.seq_svc.loadPattern("/tmp/test-samplerobot-move-base-pose", 1.0)
    hcf.seq_svc.waitInterpolation()
    checkRobotState(move_base_pose_doc)
    hcf.seq_svc.loadPattern("/tmp/test-samplerobot-reset-pose", 1.0)
    hcf.seq_svc.waitInterpolation()
    checkRobotState(reset_pose_doc)

def demoSetZmp ():
    print >> sys.stderr, "5. setZmp"
    hcf.seq_svc.setZmp(move_base_pose_doc['zmp'], 1.0);
    hcf.seq_svc.waitInterpolation();
    checkZmp(move_base_pose_doc)
    hcf.seq_svc.setZmp(reset_pose_doc['zmp'], 1.0);
    hcf.seq_svc.waitInterpolation();
    checkZmp(reset_pose_doc)

def demoSetBasePosRpy ():
    print >> sys.stderr, "6. setBasePos and setBaseRpy"
    hcf.seq_svc.setBasePos(move_base_pose_doc['waist'][0:3], 1.0);
    hcf.seq_svc.setBaseRpy(move_base_pose_doc['waist'][3:6], 1.0);
    hcf.seq_svc.waitInterpolation();
    checkWaist(move_base_pose_doc)
    hcf.seq_svc.setBasePos(reset_pose_doc['waist'][0:3], 1.0);
    hcf.seq_svc.setBaseRpy(reset_pose_doc['waist'][3:6], 1.0);
    hcf.seq_svc.waitInterpolation();
    checkWaist(reset_pose_doc)

def demoSetWrenches ():
    print >> sys.stderr, "7. setWrenches"
    hcf.seq_svc.setWrenches(move_base_pose_doc['wrenches'], 1.0);
    hcf.seq_svc.waitInterpolation();
    checkWrenches(move_base_pose_doc)
    hcf.seq_svc.setWrenches(reset_pose_doc['wrenches'], 1.0);
    hcf.seq_svc.waitInterpolation();
    checkWrenches(reset_pose_doc)

def demoSetJointAnglesOfGroup():
    print >> sys.stderr, "8. setJointAnglesOfGroup"
    hcf.seq_svc.addJointGroup('larm', ['LARM_SHOULDER_P', 'LARM_SHOULDER_R', 'LARM_SHOULDER_Y', 'LARM_ELBOW', 'LARM_WRIST_Y', 'LARM_WRIST_P', 'LARM_WRIST_R'])
    larm_pos0 = [-0.000111, 0.31129, -0.159481, -1.57079, -0.636277, 0.0, 0.0]
    larm_pos1 = [-0.000111, 0.31129, -0.159481, -0.115399, -0.636277, 0.0, 0.0]
    hcf.seq_svc.setJointAngles(reset_pose_doc['pos'], 1.0);
    hcf.seq_svc.setJointAnglesOfGroup('larm', larm_pos0, 1.0);
    hcf.seq_svc.waitInterpolationOfGroup('larm');
    p0 = list(reset_pose_doc['pos']) # copy
    for i in range(len(larm_pos0)):
        p0[i+19] = larm_pos0[i]
    checkJointAngles(p0)
    hcf.seq_svc.setJointAnglesOfGroup('larm', larm_pos1, 1.0);
    hcf.seq_svc.waitInterpolationOfGroup('larm');
    p1 = list(reset_pose_doc['pos']) # copy
    for i in range(len(larm_pos1)):
        p1[i+19] = larm_pos1[i]
    checkJointAngles(p1)
    # check override
    print >> sys.stderr, "   check override"
    hcf.seq_svc.setJointAngles(reset_pose_doc['pos'], 1.0);
    hcf.seq_svc.setJointAnglesOfGroup('larm', larm_pos0, 5.0);
    time.sleep(2.5)
    hcf.seq_svc.setJointAnglesOfGroup('larm', larm_pos1, 1.0);
    hcf.seq_svc.waitInterpolationOfGroup('larm');
    checkJointAngles(p1)
    # check clear
    if hrpsys_version < '315.5.0':
        return
    print >> sys.stderr, "   check clear"
    hcf.seq_svc.setJointAnglesOfGroup('larm', larm_pos0, 5.0);
    time.sleep(2.5)
    hcf.seq_svc.clearJointAnglesOfGroup('larm')
    checkJointAnglesBetween(p1, p0)

def demoSetJointAnglesSequenceOfGroup():
    print >> sys.stderr, "9. setJointAnglesOfGroup"
    hcf.seq_svc.addJointGroup('larm', ['LARM_SHOULDER_P', 'LARM_SHOULDER_R', 'LARM_SHOULDER_Y', 'LARM_ELBOW', 'LARM_WRIST_Y', 'LARM_WRIST_P', 'LARM_WRIST_R'])
    larm_pos0 = [-0.000111, 0.31129, -0.159481, -1.57079, -0.636277, 0.0, 0.0]
    larm_pos1 = [-0.000111, 0.31129, -0.159481, -0.115399, -0.636277, 0.0, 0.0]
    hcf.seq_svc.setJointAngles(reset_pose_doc['pos'], 1.0);
    hcf.seq_svc.setJointAnglesSequenceOfGroup('larm', [larm_pos0, larm_pos1, larm_pos0], [1.0, 1.0, 1.0]);
    hcf.seq_svc.waitInterpolationOfGroup('larm');
    p0 = list(reset_pose_doc['pos']) # copy
    for i in range(len(larm_pos0)):
        p0[i+19] = larm_pos0[i]
    checkJointAngles(p0)
    hcf.seq_svc.setJointAnglesSequenceOfGroup('larm', [larm_pos1, larm_pos0, larm_pos1], [1.0, 1.0, 1.0]);
    hcf.seq_svc.waitInterpolationOfGroup('larm');
    p1 = list(reset_pose_doc['pos']) # copy
    for i in range(len(larm_pos1)):
        p1[i+19] = larm_pos1[i]
    checkJointAngles(p1)
    # check override
    print >> sys.stderr, "   check override"
    hcf.seq_svc.setJointAngles(reset_pose_doc['pos'], 1.0);
    hcf.seq_svc.setJointAnglesSequenceOfGroup('larm', [larm_pos0, larm_pos1, larm_pos0], [1.0, 1.0, 5.0]);
    time.sleep(3.5)
    hcf.seq_svc.setJointAnglesSequenceOfGroup('larm', [larm_pos1, larm_pos0, larm_pos1], [1.0, 1.0, 1.0]);
    hcf.seq_svc.waitInterpolationOfGroup('larm');
    checkJointAngles(p1)
    # check clear
    print >> sys.stderr, "   check clear"
    hcf.seq_svc.setJointAnglesSequenceOfGroup('larm', [larm_pos0, larm_pos1, larm_pos0], [1.0, 1.0, 5.0]);
    time.sleep(3.5)
    hcf.seq_svc.clearJointAnglesOfGroup('larm')
    checkJointAnglesBetween(p1, p0)
    hcf.seq_svc.removeJointGroup('larm')

def demoSetJointAnglesSequenceFull():
    print >> sys.stderr, "10. setJointAnglesSequenceFull"
    hcf.seq_svc.setJointAnglesSequenceFull([move_base_pose_doc['pos'],reset_pose_doc['pos'],move_base_pose_doc['pos']],
                                           [move_base_pose_doc['vel'],reset_pose_doc['vel'],move_base_pose_doc['vel']],
                                           [move_base_pose_doc['torque'],reset_pose_doc['torque'],move_base_pose_doc['torque']],
                                           [move_base_pose_doc['waist'][0:3],reset_pose_doc['waist'][0:3],move_base_pose_doc['waist'][0:3]],
                                           [move_base_pose_doc['waist'][3:6],reset_pose_doc['waist'][3:6],move_base_pose_doc['waist'][3:6]],
                                           [move_base_pose_doc['waist_acc'],reset_pose_doc['waist_acc'],move_base_pose_doc['waist_acc']],
                                           [move_base_pose_doc['zmp'],reset_pose_doc['zmp'],move_base_pose_doc['zmp']],
                                           [move_base_pose_doc['wrenches'],reset_pose_doc['wrenches'],move_base_pose_doc['wrenches']],
                                           [move_base_pose_doc['optionaldata'],reset_pose_doc['optionaldata'],move_base_pose_doc['optionaldata']],
                                           [1.0,1.0,1.0]);
    hcf.waitInterpolation();
    checkRobotState(move_base_pose_doc)
    hcf.seq_svc.setJointAnglesSequenceFull([reset_pose_doc['pos']],
                                           [reset_pose_doc['vel']],
                                           [reset_pose_doc['torque']],
                                           [reset_pose_doc['waist'][0:3]],
                                           [reset_pose_doc['waist'][3:6]],
                                           [reset_pose_doc['waist_acc']],
                                           [reset_pose_doc['zmp']],
                                           [reset_pose_doc['wrenches']],
                                           [reset_pose_doc['optionaldata']],
                                           [1.0])
    hcf.waitInterpolation();
    checkRobotState(reset_pose_doc)
    # check override
    print >> sys.stderr, "   check override"
    hcf.seq_svc.setJointAnglesSequenceFull([move_base_pose_doc['pos'],reset_pose_doc['pos'],move_base_pose_doc['pos']],
                                           [move_base_pose_doc['vel'],reset_pose_doc['vel'],move_base_pose_doc['vel']],
                                           [move_base_pose_doc['torque'],reset_pose_doc['torque'],move_base_pose_doc['torque']],
                                           [move_base_pose_doc['waist'][0:3],reset_pose_doc['waist'][0:3],move_base_pose_doc['waist'][0:3]],
                                           [move_base_pose_doc['waist'][3:6],reset_pose_doc['waist'][3:6],move_base_pose_doc['waist'][3:6]],
                                           [move_base_pose_doc['waist_acc'],reset_pose_doc['waist_acc'],move_base_pose_doc['waist_acc']],
                                           [move_base_pose_doc['zmp'],reset_pose_doc['zmp'],move_base_pose_doc['zmp']],
                                           [move_base_pose_doc['wrenches'],reset_pose_doc['wrenches'],move_base_pose_doc['wrenches']],
                                           [move_base_pose_doc['optionaldata'],reset_pose_doc['optionaldata'],move_base_pose_doc['optionaldata']],
                                           [1.0,1.0,5.0]);
    time.sleep(3.5)
    hcf.seq_svc.setJointAnglesSequenceFull([reset_pose_doc['pos'],move_base_pose_doc['pos'],reset_pose_doc['pos']],
                                           [reset_pose_doc['vel'],move_base_pose_doc['vel'],reset_pose_doc['vel']],
                                           [reset_pose_doc['torque'],move_base_pose_doc['torque'],reset_pose_doc['torque']],
                                           [reset_pose_doc['waist'][0:3],move_base_pose_doc['waist'][0:3],reset_pose_doc['waist'][0:3]],
                                           [reset_pose_doc['waist'][3:6],move_base_pose_doc['waist'][3:6],reset_pose_doc['waist'][3:6]],
                                           [reset_pose_doc['waist_acc'],move_base_pose_doc['waist_acc'],reset_pose_doc['waist_acc']],
                                           [reset_pose_doc['zmp'],move_base_pose_doc['zmp'],reset_pose_doc['zmp']],
                                           [reset_pose_doc['wrenches'],move_base_pose_doc['wrenches'],reset_pose_doc['wrenches']],
                                           [reset_pose_doc['optionaldata'],move_base_pose_doc['optionaldata'],reset_pose_doc['optionaldata']],
                                           [1.0,1.0,1.0]);
    hcf.waitInterpolation()
    checkRobotState(reset_pose_doc)
    # check clear
    print >> sys.stderr, "   check clear"
    hcf.seq_svc.setJointAnglesSequenceFull([move_base_pose_doc['pos'],reset_pose_doc['pos'],move_base_pose_doc['pos']],
                                           [move_base_pose_doc['vel'],reset_pose_doc['vel'],move_base_pose_doc['vel']],
                                           [move_base_pose_doc['torque'],reset_pose_doc['torque'],move_base_pose_doc['torque']],
                                           [move_base_pose_doc['waist'][0:3],reset_pose_doc['waist'][0:3],move_base_pose_doc['waist'][0:3]],
                                           [move_base_pose_doc['waist'][3:6],reset_pose_doc['waist'][3:6],move_base_pose_doc['waist'][3:6]],
                                           [move_base_pose_doc['waist_acc'],reset_pose_doc['waist_acc'],move_base_pose_doc['waist_acc']],
                                           [move_base_pose_doc['zmp'],reset_pose_doc['zmp'],move_base_pose_doc['zmp']],
                                           [move_base_pose_doc['wrenches'],reset_pose_doc['wrenches'],move_base_pose_doc['wrenches']],
                                           [move_base_pose_doc['optionaldata'],reset_pose_doc['optionaldata'],move_base_pose_doc['optionaldata']],
                                           [1.0,1.0,5.0]);
    time.sleep(3.5)
    hcf.seq_svc.clearJointAngles()
    checkJointAnglesBetween(reset_pose_doc,move_base_pose_doc)


def demo():
    init()
    demoSetJointAngles()
    if hrpsys_version >= '315.5.0':
        demoSetJointAnglesSequence()
    demoSetJointAngle()
    demoLoadPattern()
    demoSetZmp()
    demoSetBasePosRpy()
    if hrpsys_version >= '315.2.0':
        demoSetWrenches()
    demoSetJointAnglesOfGroup()
    if hrpsys_version >= '315.5.0':
        demoSetJointAnglesSequenceOfGroup()
        demoSetJointAnglesSequenceFull()

if __name__ == '__main__':
    demo()


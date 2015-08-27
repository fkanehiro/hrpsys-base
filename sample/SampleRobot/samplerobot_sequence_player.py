#!/usr/bin/env python

from hrpsys.hrpsys_config import *
from hrpsys import OpenHRP
from hrpsys import rtm

def init ():
    global hcf, hrpsys_version
    hcf = HrpsysConfigurator()
    hcf.init ("SampleRobot(Robot)0", "$(PROJECT_DIR)/../model/sample1.wrl")
    global reset_pose_doc, move_base_pose_doc, doc
    # doc for patterns.
    #  torque and wrenches are non-realistic values, just for testing.
    dof = 29
    reset_pose_doc = {'pos':[-7.778932e-05,-0.378613,-0.00021,0.832039,-0.452564,0.000245,0.31129,-0.159481,-0.115399,-0.636277,0.0,0.0,0.0,-7.778932e-05,-0.378613,-0.00021,0.832039,-0.452564,0.000245,0.31129,0.159481,0.115399,-0.636277,0.0,0.0,0.0,0.0,0.0,0.0],
                      'zmp':[-0.00081, 1.712907e-05, -0.66815],
#                      'gsens':[0,0,0],
                      'waist':[0.000234, 0.000146, 0.66815, -0.000245, -0.000862, 0.000195],
                      'torque':[0]*dof,
                      'wrenches':[0]*24
                      }
    move_base_pose_doc = {'pos':[8.251963e-05,-0.980029,-0.000384,1.02994,-0.398115,-0.000111,0.31129,-0.159481,-0.115399,-0.636277,0.0,0.0,0.0,8.252625e-05,-0.980033,-0.000384,1.02986,-0.398027,-0.000111,0.31129,0.159481,0.115399,-0.636277,0.0,0.0,0.0,0.0,0.0,0.0],
                          'zmp':[0.302518, 0.000153, -0.562325],
#                          'gsens':[0,0,0],
                          'waist':[-0.092492, -6.260780e-05, 0.6318, -0.000205, 0.348204, 0.000268],
                          'torque':range(dof),
                          'wrenches':[1]*6+[-2]*6+[3]*6+[-4]*6
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
    return all(map(lambda x,y,z : abs(x-y)<eps or (z-x)*(y-x) > 0, arr1, arr2, arr3))

def checkJointAngles (var_doc):
    if isinstance(var_doc, list):
        p = var_doc
    else:
        p = var_doc['pos']
    ret = checkArrayEquality(rtm.readDataPort(hcf.sh.port("qOut")).data, p)
    print "  pos => ", ret

def checkJointAnglesBetween(from_doc, to_doc):
    p0 =  from_doc if isinstance(from_doc, list) else from_doc['pos']
    p1 =    to_doc if isinstance(  to_doc, list) else   to_doc['pos']
    ret = checkArrayBetween(p0, rtm.readDataPort(hcf.sh.port("qOut")).data, p1)
    print "  pos => ", ret
    assert(ret is True)

def checkZmp(var_doc):
    zmp=rtm.readDataPort(hcf.sh.port("zmpOut")).data
    ret = checkArrayEquality([zmp.x, zmp.y, zmp.z], var_doc['zmp'])
    print "  zmp => ", ret
    assert(ret is True)

def checkWaist(var_doc):
    bpos=rtm.readDataPort(hcf.sh.port("basePosOut")).data
    brpy=rtm.readDataPort(hcf.sh.port("baseRpyOut")).data
    ret = checkArrayEquality([bpos.x, bpos.y, bpos.z, brpy.r, brpy.p, brpy.y], var_doc['waist'])
    print "  waist => ", ret
    assert(ret is True)

def checkTorque (var_doc):
    ret = checkArrayEquality(rtm.readDataPort(hcf.sh.port("tqOut")).data, var_doc['torque'])
    print "  torque => ", ret
    assert(ret is True)

def checkWrenches (var_doc):
    ret = checkArrayEquality(reduce(lambda x,y:x+y, map(lambda fs : rtm.readDataPort(hcf.sh.port(fs+"Out")).data, ['lfsensor', 'rfsensor', 'lhsensor', 'rhsensor'])), var_doc['wrenches'])
    print "  wrenches => ", ret
    assert(ret is True)

def checkRobotState (var_doc):
    checkJointAngles(var_doc)
    checkZmp(var_doc)
    checkWaist(var_doc)
    checkTorque(var_doc)
    if hrpsys_version >= '315.2.0':
        checkWrenches(var_doc)

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

if __name__ == '__main__':
    demo()


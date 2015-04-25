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
    global hcf
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

def dumpLoadPatternTestFile (basename, var_doc, tm):
    for key in var_doc.keys():
        f=open(basename+"."+key, "w")
        dumpstr=" ".join(map(str, [tm]+var_doc[key]))
        f.writelines(dumpstr)
        f.close()

def checkArrayEquality (arr1, arr2, eps=1e-7):
    return all(map(lambda x,y : abs(x-y)<eps, arr1, arr2))

def checkJointAngles (var_doc):
    print "  pos => ", checkArrayEquality(rtm.readDataPort(hcf.sh.port("qOut")).data, var_doc['pos'])

def checkZmp(var_doc):
    zmp=rtm.readDataPort(hcf.sh.port("zmpOut")).data
    print "  zmp => ", checkArrayEquality([zmp.x, zmp.y, zmp.z], var_doc['zmp'])

def checkWaist(var_doc):
    bpos=rtm.readDataPort(hcf.sh.port("basePosOut")).data
    brpy=rtm.readDataPort(hcf.sh.port("baseRpyOut")).data
    print "  waist => ", checkArrayEquality([bpos.x, bpos.y, bpos.z, brpy.r, brpy.p, brpy.y], var_doc['waist'])

def checkTorque (var_doc):
    print "  torque => ", checkArrayEquality(rtm.readDataPort(hcf.sh.port("tqOut")).data, var_doc['torque'])

def checkWrenches (var_doc):
    print "  wrenches => ", checkArrayEquality(reduce(lambda x,y:x+y, map(lambda fs : rtm.readDataPort(hcf.sh.port(fs+"Out")).data, ['lfsensor', 'rfsensor', 'lhsensor', 'rhsensor'])), var_doc['wrenches'])

def checkRobotState (var_doc):
    checkJointAngles(var_doc)
    checkZmp(var_doc)
    checkWaist(var_doc)
    checkTorque(var_doc)
    checkWrenches(var_doc)

# demo functions
def demoSetJointAngles():
    print "1. setJointAngles"
    hcf.seq_svc.setJointAngles(move_base_pose_doc['pos'], 1.0);
    hcf.seq_svc.waitInterpolation();
    checkJointAngles(move_base_pose_doc)
    hcf.seq_svc.setJointAngles(reset_pose_doc['pos'], 1.0);
    hcf.seq_svc.waitInterpolation();
    checkJointAngles(reset_pose_doc)

def demoSetJointAngle():
    print "2. setJointAngle"
    hcf.seq_svc.setJointAngle("WAIST_R", 10*3.14159/180.0, 1.0);
    hcf.seq_svc.waitInterpolation();
    hcf.seq_svc.setJointAngle("WAIST_R", 0*3.14159/180.0, 1.0);
    hcf.seq_svc.waitInterpolation();

def demoLoadPattern():
    print "3. loadPattern"
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
    print "4. setZmp"
    hcf.seq_svc.setZmp(move_base_pose_doc['zmp'], 1.0);
    hcf.seq_svc.waitInterpolation();
    checkZmp(move_base_pose_doc)
    hcf.seq_svc.setZmp(reset_pose_doc['zmp'], 1.0);
    hcf.seq_svc.waitInterpolation();
    checkZmp(reset_pose_doc)

def demoSetBasePosRpy ():
    print "4. setBasePos and setBaseRpy"
    hcf.seq_svc.setBasePos(move_base_pose_doc['waist'][0:3], 1.0);
    hcf.seq_svc.setBaseRpy(move_base_pose_doc['waist'][3:6], 1.0);
    hcf.seq_svc.waitInterpolation();
    checkWaist(move_base_pose_doc)
    hcf.seq_svc.setBasePos(reset_pose_doc['waist'][0:3], 1.0);
    hcf.seq_svc.setBaseRpy(reset_pose_doc['waist'][3:6], 1.0);
    hcf.seq_svc.waitInterpolation();
    checkWaist(reset_pose_doc)

def demoSetWrenches ():
    print "5. setWrenches"
    hcf.seq_svc.setWrenches(move_base_pose_doc['wrenches'], 1.0);
    hcf.seq_svc.waitInterpolation();
    checkWrenches(move_base_pose_doc)
    hcf.seq_svc.setWrenches(reset_pose_doc['wrenches'], 1.0);
    hcf.seq_svc.waitInterpolation();
    checkWrenches(reset_pose_doc)

def demo():
    init()
    demoSetJointAngles()
    demoSetJointAngle()
    demoLoadPattern()
    demoSetZmp()
    demoSetBasePosRpy()
    demoSetWrenches()

if __name__ == '__main__':
    demo()


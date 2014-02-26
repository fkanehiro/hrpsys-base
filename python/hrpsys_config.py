#!/usr/bin/env python

import os
import rtm

from rtm import *
from OpenHRP import *
from hrpsys import * # load ModelLoader

import socket
import time

# copy from transformations.py, Christoph Gohlke, The Regents of the University of California

import numpy
# map axes strings to/from tuples of inner axis, parity, repetition, frame
_AXES2TUPLE = {
    'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
    'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
    'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
    'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
    'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
    'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
    'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
    'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

# axis sequences for Euler angles
_NEXT_AXIS = [1, 2, 0, 1]

# epsilon for testing whether a number is close to zero
_EPS = numpy.finfo(float).eps * 4.0

def euler_matrix(ai, aj, ak, axes='sxyz'):
    """Return homogeneous rotation matrix from Euler angles and axis sequence.

    ai, aj, ak : Euler's roll, pitch and yaw angles
    axes : One of 24 axis sequences as string or encoded tuple

    >>> R = euler_matrix(1, 2, 3, 'syxz')
    >>> numpy.allclose(numpy.sum(R[0]), -1.34786452)
    True
    >>> R = euler_matrix(1, 2, 3, (0, 1, 0, 1))
    >>> numpy.allclose(numpy.sum(R[0]), -0.383436184)
    True
    >>> ai, aj, ak = (4.0*math.pi) * (numpy.random.random(3) - 0.5)
    >>> for axes in _AXES2TUPLE.keys():
    ...    R = euler_matrix(ai, aj, ak, axes)
    >>> for axes in _TUPLE2AXES.keys():
    ...    R = euler_matrix(ai, aj, ak, axes)

    """
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes]
    except (AttributeError, KeyError):
        _ = _TUPLE2AXES[axes]
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]

    if frame:
        ai, ak = ak, ai
    if parity:
        ai, aj, ak = -ai, -aj, -ak

    si, sj, sk = math.sin(ai), math.sin(aj), math.sin(ak)
    ci, cj, ck = math.cos(ai), math.cos(aj), math.cos(ak)
    cc, cs = ci*ck, ci*sk
    sc, ss = si*ck, si*sk

    M = numpy.identity(4)
    if repetition:
        M[i, i] = cj
        M[i, j] = sj*si
        M[i, k] = sj*ci
        M[j, i] = sj*sk
        M[j, j] = -cj*ss+cc
        M[j, k] = -cj*cs-sc
        M[k, i] = -sj*ck
        M[k, j] = cj*sc+cs
        M[k, k] = cj*cc-ss
    else:
        M[i, i] = cj*ck
        M[i, j] = sj*sc-cs
        M[i, k] = sj*cc+ss
        M[j, i] = cj*sk
        M[j, j] = sj*ss+cc
        M[j, k] = sj*cs-sc
        M[k, i] = -sj
        M[k, j] = cj*si
        M[k, k] = cj*ci
    return M

def euler_from_matrix(matrix, axes='sxyz'):
    """Return Euler angles from rotation matrix for specified axis sequence.

    axes : One of 24 axis sequences as string or encoded tuple

    Note that many Euler angle triplets can describe one matrix.

    >>> R0 = euler_matrix(1, 2, 3, 'syxz')
    >>> al, be, ga = euler_from_matrix(R0, 'syxz')
    >>> R1 = euler_matrix(al, be, ga, 'syxz')
    >>> numpy.allclose(R0, R1)
    True
    >>> angles = (4.0*math.pi) * (numpy.random.random(3) - 0.5)
    >>> for axes in _AXES2TUPLE.keys():
    ...    R0 = euler_matrix(axes=axes, *angles)
    ...    R1 = euler_matrix(axes=axes, *euler_from_matrix(R0, axes))
    ...    if not numpy.allclose(R0, R1): print axes, "failed"

    """
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (AttributeError, KeyError):
        _ = _TUPLE2AXES[axes]
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]

    M = numpy.array(matrix, dtype=numpy.float64, copy=False)[:3, :3]
    if repetition:
        sy = math.sqrt(M[i, j]*M[i, j] + M[i, k]*M[i, k])
        if sy > _EPS:
            ax = math.atan2( M[i, j],  M[i, k])
            ay = math.atan2( sy,       M[i, i])
            az = math.atan2( M[j, i], -M[k, i])
        else:
            ax = math.atan2(-M[j, k],  M[j, j])
            ay = math.atan2( sy,       M[i, i])
            az = 0.0
    else:
        cy = math.sqrt(M[i, i]*M[i, i] + M[j, i]*M[j, i])
        if cy > _EPS:
            ax = math.atan2( M[k, j],  M[k, k])
            ay = math.atan2(-M[k, i],  cy)
            az = math.atan2( M[j, i],  M[i, i])
        else:
            ax = math.atan2(-M[j, k],  M[j, j])
            ay = math.atan2(-M[k, i],  cy)
            az = 0.0

    if parity:
        ax, ay, az = -ax, -ay, -az
    if frame:
        ax, az = az, ax
    return ax, ay, az

# class for configure hrpsys RTCs and ports
#   In order to specify robot-dependent code, please inherit this HrpsysConfigurator
class HrpsysConfigurator:

    # RobotHardware
    rh = None
    rh_svc = None
    ep_svc = None

    # SequencePlayer
    seq = None
    seq_svc = None

    # StateHolder
    sh = None
    sh_svc = None

    # ForwardKinematics
    fk = None
    fk_svc = None

    tf = None # TorqueFilter
    kf = None # KalmanFilter
    vs = None # VirtualForceSensor
    afs = None # AbsoluteForceSensor
    ic = None # ImpedanceController
    abc = None # AutoBalancer
    st = None # Stabilizer

    # CollisionDetector
    co = None
    co_svc = None

    # GraspController
    gc = None
    gc_svc = None

    # SoftErrorLimiter
    el = None
    el_svc = None

    # ThermoEstimator
    te = None
    te_svc = None

    # ThermoLimiter
    tl = None
    tl_svc = None

    # TorqueController
    tc = None
    tc_svc = None

    # DataLogger
    log = None
    log_svc = None

    # rtm manager
    ms = None

    # HGController(Simulation)
    hgc = None

    # flag isSimulation?
    simulation_mode = None

    # sensors
    sensors = None

    # public method
    def connectComps(self):
        if self.rh == None or self.seq == None or self.sh == None or self.fk == None:
            print self.configurator_name, "\e[1;31m connectComps : hrpsys requries rh, seq, sh and fk, please check rtcd.conf or rtcd arguments\e[0m"
            return
        # connection for reference joint angles
        tmp_contollers = self.getJointAngleControllerList()
        if len(tmp_contollers) > 0:
            connectPorts(self.sh.port("qOut"),  tmp_contollers[0].port("qRef"))
            for i in range(len(tmp_contollers)-1):
                connectPorts(tmp_contollers[i].port("q"), tmp_contollers[i+1].port("qRef"))
            if self.simulation_mode:
                connectPorts(tmp_contollers[-1].port("q"),  self.hgc.port("qIn"))
                connectPorts(self.hgc.port("qOut"), self.rh.port("qRef"))
            else :
                connectPorts(tmp_contollers[-1].port("q"),  self.rh.port("qRef"))
        else:
            if self.simulation_mode :
                connectPorts(self.sh.port("qOut"),  self.hgc.port("qIn"))
                connectPorts(self.hgc.port("qOut"), self.rh.port("qRef"))
            else:
                connectPorts(self.sh.port("qOut"),  self.rh.port("qRef"))

        # connection for kf
        if self.kf:
            #   currently use first acc and rate sensors for kf
            s_acc=filter(lambda s : s.type == 'Acceleration', self.sensors)
            if (len(s_acc)>0) and self.rh.port(s_acc[0].name) != None: # check existence of sensor ;; currently original HRP4C.xml has different naming rule of gsensor and gyrometer
                connectPorts(self.rh.port(s_acc[0].name), self.kf.port('acc'))
            s_rate=filter(lambda s : s.type == 'RateGyro', self.sensors)
            if (len(s_rate)>0) and self.rh.port(s_rate[0].name) != None: # check existence of sensor ;; currently original HRP4C.xml has different naming rule of gsensor and gyrometer
                connectPorts(self.rh.port(s_rate[0].name), self.kf.port("rate"))
            connectPorts(self.seq.port("accRef"), self.kf.port("accRef"))

        # connection for rh
        if self.rh.port("servoState") != None:
            if self.te and self.el:
                connectPorts(self.rh.port("servoState"), self.te.port("servoStateIn"))
                connectPorts(self.te.port("servoStateOut"), self.el.port("servoStateIn"))
            elif self.el:
                connectPorts(self.rh.port("servoState"), self.el.port("servoStateIn"))
            elif self.te:
                connectPorts(self.rh.port("servoState"), self.te.port("servoStateIn"))

        # connection for sh, seq, fk
        connectPorts(self.rh.port("q"), [self.sh.port("currentQIn"), self.fk.port("q")]) # connection for actual joint angles
        connectPorts(self.sh.port("qOut"),  self.fk.port("qRef"))
        connectPorts(self.seq.port("qRef"), self.sh.port("qIn"))
        connectPorts(self.seq.port("tqRef"), self.sh.port("tqIn"))
        connectPorts(self.seq.port("basePos"), self.sh.port("basePosIn"))
        connectPorts(self.seq.port("baseRpy"), self.sh.port("baseRpyIn"))
        connectPorts(self.seq.port("zmpRef"),  self.sh.port("zmpIn"))
        connectPorts(self.sh.port("basePosOut"), [self.seq.port("basePosInit"), self.fk.port("basePosRef")])
        connectPorts(self.sh.port("baseRpyOut"), [self.seq.port("baseRpyInit"), self.fk.port("baseRpyRef")])
        connectPorts(self.sh.port("qOut"), self.seq.port("qInit"))

        # connection for st
        if rtm.findPort(self.rh.ref, "lfsensor") and rtm.findPort(self.rh.ref, "rfsensor") and self.st:
            connectPorts(self.rh.port("lfsensor"), self.st.port("forceL"))
            connectPorts(self.rh.port("rfsensor"), self.st.port("forceR"))
            connectPorts(self.kf.port("rpy"), self.st.port("rpy"))
            connectPorts(self.abc.port("zmpRef"), self.st.port("zmpRef"))
            connectPorts(self.abc.port("baseRpy"), self.st.port("baseRpyIn"))
            connectPorts(self.abc.port("basePos"), self.st.port("basePosIn"))
        if self.ic and self.abc:
            for sen in filter(lambda x : x.type == "Force", self.sensors):
                connectPorts(self.ic.port("ref_"+sen.name), self.abc.port("ref_"+sen.name))

        #  actual force sensors
        if self.afs and self.kf:
            #connectPorts(self.kf.port("rpy"), self.ic.port("rpy"))
            connectPorts(self.kf.port("rpy"), self.afs.port("rpy"))
            connectPorts(self.rh.port("q"), self.afs.port("qCurrent"))
            for sen in filter(lambda x : x.type == "Force", self.sensors):
                connectPorts(self.rh.port(sen.name), self.afs.port(sen.name))
                if self.ic:
                    connectPorts(self.afs.port("off_"+sen.name), self.ic.port(sen.name))
        # connection for ic
        if self.ic:
            connectPorts(self.rh.port("q"), self.ic.port("qCurrent"))
        # connection for tf
        if self.tf:
            # connection for actual torques
            if rtm.findPort(self.rh.ref, "tau") != None:
                connectPorts(self.rh.port("tau"), self.tf.port("tauIn"))
            connectPorts(self.rh.port("q"), self.tf.port("qCurrent"))
        # connection for vs
        if self.vs:
            connectPorts(self.rh.port("q"), self.vs.port("qCurrent"))
            connectPorts(self.tf.port("tauOut"), self.vs.port("tauIn"))
            #  virtual force sensors
            if self.ic:
                for vfp in filter(lambda x : str.find(x, 'v') >= 0 and str.find(x, 'sensor') >= 0, self.vs.ports.keys()):
                    connectPorts(self.vs.port(vfp), self.ic.port(vfp))
        # connection for co
        if self.co:
            connectPorts(self.rh.port("q"), self.co.port("qCurrent"))

        # connection for gc
        if self.gc:
            connectPorts(self.rh.port("q"), self.gc.port("qCurrent")) # other connections
            tmp_controller = self.getJointAngleControllerList()
            index = tmp_controller.index(self.gc)
            if index == 0:
                connectPorts(self.sh.port("qOut"), self.gc.port("qIn"))
            else:
                connectPorts(tmp_controller[index - 1].port("q"), self.gc.port("qIn"))

        # connection for te
        if self.te:
            if self.tf:
                connectPorts(self.tf.port("tauOut"), self.te.port("tauIn"))
            else:
                connectPorts(self.rh.port("tau"), self.te.port("tauIn"))
            # sevoStateIn is connected above
            
        # connection for tl
        if self.tl:
            if self.tf:
                connectPorts(self.tf.port("tauOut"), self.tl.port("tauIn"))
            else:
                connectPorts(self.rh.port("tau"), self.tl.port("tauIn"))
            if self.te:
                connectPorts(self.te.port("tempOut"), self.tl.port("tempIn"))
            connectPorts(self.rh.port("q"), self.tl.port("qCurrentIn"))
            # qRef is connected as joint angle controller

        # connection for tc
        if self.tc:
            connectPorts(self.rh.port("q"), self.tc.port("qCurrent"))
            if self.tf:
                connectPorts(self.tf.port("tauOut"), self.tc.port("tauCurrent"))
            else:
                connectPorts(self.rh.port("tau"), self.tc.port("tauCurrent"))
            if self.tl:
                connectPorts(self.tl.port("tauMax"), self.tc.port("tauMax"))

        # connection for el
        if self.el:
            connectPorts(self.rh.port("q"), self.el.port("qCurrent"))

    def activateComps(self):
        rtcList = self.getRTCInstanceList()
        rtm.serializeComponents(rtcList)
        for r in rtcList:
            r.start()

    def createComp(self, compName, instanceName):
        self.ms.load(compName)
        comp = self.ms.create(compName, instanceName)
        print self.configurator_name, "create Comp -> ", compName, " : ", comp
        if comp == None:
            raise RuntimeError("Cannot create component: " + compName)
        if comp.service("service0"):
            comp_svc = narrow(comp.service("service0"), compName+"Service")
            print self.configurator_name, "create CompSvc -> ", compName, "Service : ", comp_svc
            return [comp, comp_svc]
        else:
            return [comp, None]

    def createComps(self):
        for rn in self.getRTCList():
            rn2='self.'+rn[0]
            if eval(rn2) == None:
                create_str="[self."+rn[0]+", self."+rn[0]+"_svc] = self.createComp(\""+rn[1]+"\",\""+rn[0]+"\")"
                print self.configurator_name, "  eval : ", create_str
                exec(create_str)

    # public method to configure all RTCs to be activated on rtcd
    def getRTCList(self):
        return [
            ['seq', "SequencePlayer"],
            ['sh', "StateHolder"],
            ['fk', "ForwardKinematics"],
            ['tf', "TorqueFilter"],
            ['kf', "KalmanFilter"],
            ['vs', "VirtualForceSensor"],
            ['afs', "AbsoluteForceSensor"],
            ['ic', "ImpedanceController"],
            ['abc', "AutoBalancer"],
            ['st', "Stabilizer"],
            ['co', "CollisionDetector"],
            ['tc', "TorqueController"],
            #['te', "ThermoEstimator"],
            #['tl', "ThermoLimiter"],
            ['el', "SoftErrorLimiter"],
            ['log', "DataLogger"]
            ]

    def getJointAngleControllerList(self):
        controller_list = [self.ic, self.gc, self.abc, self.st, self.co, self.tc, self.el]
        return filter(lambda c : c != None, controller_list) # only return existing controllers

    def getRTCInstanceList(self):
        ret = [self.rh]
        for r in map(lambda x : 'self.'+x[0], self.getRTCList()):
            ret.append(eval(r))
        return ret

    # public method to get bodyInfo
    def getBodyInfo(self, url):
        import CosNaming
        obj = rtm.rootnc.resolve([CosNaming.NameComponent('ModelLoader', '')])
        mdlldr = obj._narrow(ModelLoader_idl._0_OpenHRP__POA.ModelLoader)
        print self.configurator_name, "  bodyinfo URL = file://"+url
        return mdlldr.getBodyInfo("file://"+url)

    # public method to get sensors list
    def getSensors(self, url):
        if url=='':
            return []
        else:
            return sum(map(lambda x : x.sensors, filter(lambda x : len(x.sensors) > 0, self.getBodyInfo(url)._get_links())), [])  # sum is for list flatten

    def connectLoggerPort(self, artc, sen_name, log_name=None):
        log_name = log_name if log_name else artc.name() + "_" + sen_name
        if artc and rtm.findPort(artc.ref, sen_name) != None:
            sen_type = rtm.dataTypeOfPort(artc.port(sen_name)).split("/")[1].split(":")[0]
            if rtm.findPort(self.log.ref, log_name) == None:
                print self.configurator_name, "  setupLogger : record type =", sen_type, ", name = ", sen_name, " to ", log_name
                self.log_svc.add(sen_type, log_name)
            else:
                print self.configurator_name, "  setupLogger : ", sen_name, " arleady exists in DataLogger"
            connectPorts(artc.port(sen_name), self.log.port(log_name))

    # public method to configure default logger data ports
    def setupLogger(self):
        if self.log == None:
            print self.configurator_name, "\e[1;31m  setupLogger : self.log is not defined, please check rtcd.conf or rtcd arguments\e[0m"
            return
        #
        for pn in ['q', 'tau']:
            self.connectLoggerPort(self.rh, pn)
        # sensor logger ports
        print self.configurator_name, "sensor names for DataLogger"
        for sen in self.sensors:
            self.connectLoggerPort(self.rh, sen.name)
        #
        if self.kf != None:
            self.connectLoggerPort(self.kf, 'rpy')
        if self.seq != None:
            self.connectLoggerPort(self.seq, 'qRef')
        if self.sh != None:
            self.connectLoggerPort(self.sh, 'qOut')
            self.connectLoggerPort(self.sh, 'tqOut')
            self.connectLoggerPort(self.sh, 'basePosOut')
            self.connectLoggerPort(self.sh, 'baseRpyOut')
        if self.rh != None:
            self.connectLoggerPort(self.rh, 'emergencySignal', 'emergencySignal')

    def waitForRTCManager(self, managerhost=nshost):
        self.ms = None
        while self.ms == None :
            time.sleep(1);
            if managerhost == "localhost":
                managerhost = socket.gethostname()
            self.ms = rtm.findRTCmanager(managerhost)
            print self.configurator_name, "wait for RTCmanager : ", managerhost

    def waitForRobotHardware(self, robotname="Robot"):
        self.rh = None
        timeout_count = 0;
        # wait for simulator or RobotHardware setup which sometime takes a long time
        while self.rh == None and timeout_count < 10: # <- time out limit
            time.sleep(1);
            self.rh = rtm.findRTC("RobotHardware0")
            if not self.rh:
                self.rh = rtm.findRTC(robotname)
            print self.configurator_name, "wait for", robotname, " : ",self.rh, "(timeout ", timeout_count, " < 10)"
            timeout_count += 1

        if not self.rh:
            print self.configurator_name, "Could not find ", robotname
            if self.ms:
                print self.configurator_name, "Candidates are .... ", [x.name()  for x in self.ms.get_components()]
            print self.configurator_name, "Exitting.... ", robotname
            exit(1)

        print self.configurator_name, "findComps -> RobotHardware : ",self.rh

    def checkSimulationMode(self):
        # distinguish real robot from simulation by using "servoState" port
        if rtm.findPort(self.rh.ref, "servoState") == None:
            self.hgc = findRTC("HGcontroller0")
            self.simulation_mode = True
        else:
            self.simulation_mode = False
            self.rh_svc = narrow(self.rh.service("service0"), "RobotHardwareService")
            self.ep_svc = narrow(self.rh.ec, "ExecutionProfileService")

        print self.configurator_name, "simulation_mode : ", self.simulation_mode

    def waitForRTCManagerAndRoboHardware(self, robotname="Robot", managerhost=nshost):
        self.waitForRTCManager(managerhost)
        self.waitForRobotHardware(robotname)
        self.checkSimulationMode()

    def findModelLoader(self):
        try:
            return rtm.findObject("ModelLoader")
        except:
            return None

    def waitForModelLoader(self):
        while self.findModelLoader() == None: # seq uses modelloader
            print self.configurator_name, "wait for ModelLoader"
            time.sleep(3);

    ##
    ## service interface for RTC component
    ##
    def goActual(self):
        self.sh_svc.goActual()

    def setJointAngle(self, jname, angle, tm):
        radangle = angle/180.0*math.pi
        return self.seq_svc.setJointAngle(jname, radangle, tm)

    def setJointAngles(self, angles, tm):
        ret = []
        for angle in angles:
            angles.append(angle/180.0*math.pi)
        return self.seq_svc.setJointAngles(ret, tm)

    def setJointAnglesOfGroup(self, gname, pose, tm, wait=True):
        angles = [x/180.0*math.pi for x in pose]
        ret = self.seq_svc.setJointAnglesOfGroup(gname, angles, tm)
        if wait:
            self.waitInterpolationOfGroup(gname)
        return ret

    def loadPattern(self, fname, tm):
        return self.seq_svc.loadPattern(fname, tm)

    def waitInterpolation(self):
        self.seq_svc.waitInterpolation()

    def waitInterpolationOfGroup(self, gname):
        self.seq_svc.waitInterpolationOfGroup(gname)

    def getJointAngles(self):
        return [x*180.0/math.pi for x in self.sh_svc.getCommand().jointRefs]

    def getCurrentPose(self,lname):
        pose = self.fk_svc.getCurrentPose(lname)
        if not pose[0] :
            raise RuntimeError("Could not find reference : " + lname)
        return pose[1].data

    def getCurrentPosition(self,lname):
        pose = self.getCurrentPose(lname)
        return [pose[3],pose[7],pose[11]]

    def getCurrentRotation(self,lname):
        pose = self.getCurrentPose(lname)
        return [pose[0:3],pose[4:7],pose[8:11]]

    def getCurrentRPY(self,lname):
        return euler_from_matrix(self.getCurrentRotation(lname),'sxyz')

    def getReferencePose(self,lname):
        pose = self.fk_svc.getReferencePose(lname)
        if not pose[0] :
            raise RuntimeError("Could not find reference : " + lname)
        return pose[1].data

    def getReferencePosition(self,lname):
        pose = self.getReferencePose(lname)
        return [pose[3],pose[7],pose[11]]

    def getReferenceRotation(self,lname):
        pose = self.getReferencePose(lname)
        return [pose[0:3],pose[4:7],pose[8:11]]

    def getReferenceRPY(self,lname):
        return euler_from_matrix(self.getReferenceRotation(lname),'sxyz')

    def setTargetPose(self, gname, pos, rpy, tm, frame_name=None) :
        print gname, frame_name, pos, rpy, tm
        if frame_name :
            gname = gname + ':' + frame_name
        return self.seq_svc.setTargetPose(gname, pos, rpy, tm)

    def setTargetPoseRelative(self, gname, eename, dx=0, dy=0, dz=0,
dr=0, dp=0, dw=0, tm=10, wait=True):
        '''
        All d* arguments are in meter.
        @param gname: Name of the link group.
        @param eename: Name of the link.
        '''
        self.waitInterpolationOfGroup(gname)
        #curPose = self.getCurrentPose(eename)
        posRef = None
        rpyRef = None
        ret, tds = self.fk_svc.getCurrentPose(eename)
        if ret:
            posRef = numpy.array([tds.data[3], tds.data[7], tds.data[11]])
            rpyRef = numpy.array(euler_from_matrix([tds.data[0:3],
tds.data[4:7], tds.data[8:11]], 'sxyz'))
            posRef += [dx, dy, dz]
            rpyRef += [dr, dp, dw]
            print posRef, rpyRef
            ret = self.setTargetPose(gname, list(posRef), list(rpyRef), tm)
            if ret and wait:
                self.waitInterpolationOfGroup(gname)
            return ret
        return False

    def clear(self):
        self.seq_svc.clear()

    def clearOfGroup(self, gname, tm=0.0):
        self.seq_svc.clearOfGroup(gname, tm)

    def saveLog(self, fname='sample'):
        self.log_svc.save(fname)
        print self.configurator_name, "saved data to ",fname

    def clearLog(self):
        self.log_svc.clear()

    def lengthDigitalInput(self):
        return self.rh_svc.lengthDigitalInput()

    def lengthDigitalOutput(self):
        return self.rh_svc.lengthDigitalOutput()

    def writeDigitalOutput(self, dout):
        doutBitLength = self.lengthDigitalOutput()*8
        if len(dout) < doutBitLength:
            for i in range(doutBitLength-len(dout)):
                dout.append(0)
        outStr = ''
        for i in range(0, len(dout), 8):
            oneChar = 0
            for j in range(8):
                if dout[i+j]:
                    oneChar += 1<<j
            outStr = outStr + chr(oneChar)

        # octet sequences are mapped to strings in omniorbpy
        return self.rh_svc.writeDigitalOutput(outStr)

    def writeDigitalOutputWithMask(self, dout, mask):
        doutBitLength = self.lengthDigitalOutput()*8
        if len(dout) < doutBitLength and \
               len(mask) < doutBitLength and \
               len(dout) == len(mask):
            for i in range(doutBitLength-len(dout)):
                dout.append(0)
                mask.append(0)
        outStr = ''
        outMsk = ''
        for i in range(0, len(dout), 8):
            oneChar = 0
            oneMask = 0
            for j in range(8):
                if dout[i+j]:
                    oneChar += 1<<j
                if mask[i+j]:
                    oneMask += 1<<j
            outStr = outStr + chr(oneChar)
            outMsk = outMsk + chr(oneMask)

        # octet sequences are mapped to strings in omniorbpy
        return self.rh_svc.writeDigitalOutputWithMask(outStr, outMsk)


    def readDigitalInput(self):
        ret, din = self.rh_svc.readDigitalInput()
        retList = []
        for item in din:
            for i in range(8):
                if (ord(item)>>i)&1:
                    retList.append(1)
                else:
                    retList.append(0)
        return retList

    ###
    ### initialize
    ###

    def init(self, robotname="Robot", url=""):
        print self.configurator_name, "waiting ModelLoader"
        self.waitForModelLoader()
        print self.configurator_name, "start hrpsys"

        print self.configurator_name, "finding RTCManager and RobotHardware"
        self.waitForRTCManagerAndRoboHardware(robotname)
        self.sensors = self.getSensors(url)

        print self.configurator_name, "creating components"
        self.createComps()

        print self.configurator_name, "connecting components"
        self.connectComps()

        print self.configurator_name, "activating components"
        self.activateComps()

        self.setupLogger()
        print self.configurator_name, "setup logger done"

        print self.configurator_name, '\033[32minitialized successfully\033[0m'

    def __init__(self, cname="[hrpsys.py] "):
        initCORBA()
        self.configurator_name = cname


if __name__ == '__main__':
    hcf = HrpsysConfigurator()
    if len(sys.argv) > 2 :
        hcf.init(sys.argv[1], sys.argv[2])
    elif len(sys.argv) > 1 :
        hcf.init(sys.argv[1])
    else :
        hcf.init()




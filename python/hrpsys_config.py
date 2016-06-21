#!/usr/bin/env python

import os
import rtm

from rtm import *
from OpenHRP import *
from hrpsys import *  # load ModelLoader
from hrpsys import ImpedanceControllerService_idl
from waitInput import waitInputConfirm

import socket
import time
import subprocess

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
    j = _NEXT_AXIS[i + parity]
    k = _NEXT_AXIS[i - parity + 1]

    if frame:
        ai, ak = ak, ai
    if parity:
        ai, aj, ak = -ai, -aj, -ak

    si, sj, sk = math.sin(ai), math.sin(aj), math.sin(ak)
    ci, cj, ck = math.cos(ai), math.cos(aj), math.cos(ak)
    cc, cs = ci * ck, ci * sk
    sc, ss = si * ck, si * sk

    M = numpy.identity(4)
    if repetition:
        M[i, i] = cj
        M[i, j] = sj * si
        M[i, k] = sj * ci
        M[j, i] = sj * sk
        M[j, j] = -cj * ss + cc
        M[j, k] = -cj * cs - sc
        M[k, i] = -sj * ck
        M[k, j] = cj * sc + cs
        M[k, k] = cj * cc - ss
    else:
        M[i, i] = cj * ck
        M[i, j] = sj * sc - cs
        M[i, k] = sj * cc + ss
        M[j, i] = cj * sk
        M[j, j] = sj * ss + cc
        M[j, k] = sj * cs - sc
        M[k, i] = -sj
        M[k, j] = cj * si
        M[k, k] = cj * ci
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
    j = _NEXT_AXIS[i + parity]
    k = _NEXT_AXIS[i - parity + 1]

    M = numpy.array(matrix, dtype=numpy.float64, copy=False)[:3, :3]
    if repetition:
        sy = math.sqrt(M[i, j] * M[i, j] + M[i, k] * M[i, k])
        if sy > _EPS:
            ax = math.atan2(M[i, j], M[i, k])
            ay = math.atan2(sy, M[i, i])
            az = math.atan2(M[j, i], -M[k, i])
        else:
            ax = math.atan2(-M[j, k], M[j, j])
            ay = math.atan2(sy, M[i, i])
            az = 0.0
    else:
        cy = math.sqrt(M[i, i] * M[i, i] + M[j, i] * M[j, i])
        if cy > _EPS:
            ax = math.atan2(M[k, j], M[k, k])
            ay = math.atan2(-M[k, i], cy)
            az = math.atan2(M[j, i], M[i, i])
        else:
            ax = math.atan2(-M[j, k], M[j, j])
            ay = math.atan2(-M[k, i], cy)
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
    rh_version = None

    # SequencePlayer
    seq = None
    seq_svc = None
    seq_version = None

    # StateHolder
    sh = None
    sh_svc = None
    sh_version = None

    # ServoController
    sc= None
    sc_svc = None
    sc_version = None

    # ForwardKinematics
    fk = None
    fk_svc = None
    fk_version = None

    tf = None  # TorqueFilter
    kf = None  # KalmanFilter
    vs = None  # VirtualForceSensor
    rmfo = None  # RemoveForceSensorLinkOffset
    ic = None  # ImpedanceController
    abc = None  # AutoBalancer
    st = None  # Stabilizer

    tf_version = None
    kf_version = None
    vs_version = None
    rmfo_version = None
    ic_version = None
    abc_version = None
    st_version = None

    # EmergencyStopper
    es = None
    es_svc = None
    es_version = None

    # EmergencyStopper (HardEmergencyStopper)
    hes = None
    hes_svc = None
    hes_version = None

    # CollisionDetector
    co = None
    co_svc = None
    co_version = None

    # GraspController
    gc = None
    gc_svc = None
    gc_version = None

    # SoftErrorLimiter
    el = None
    el_svc = None
    el_version = None

    # ThermoEstimator
    te = None
    te_svc = None
    te_version = None

    # ThermoLimiter
    tl = None
    tl_svc = None
    tl_version = None

    # TorqueController
    tc = None
    tc_svc = None
    tc_version = None

    # DataLogger
    log = None
    log_svc = None
    log_version = None
    log_use_owned_ec = False

    # Beeper
    bp = None
    bp_svc = None
    bp_version = None

    # ReferenceForceUpdater
    rfu = None
    rfu_svc = None
    rfu_version = None

    # rtm manager
    ms = None

    # HGController(Simulation)
    hgc = None

    # PDController(Simulation)
    pdc = None

    # flag isSimulation?
    simulation_mode = None

    # sensors
    sensors = None

    # for setSelfGroups
    Groups = []  # [['torso', ['CHEST_JOINT0']], ['head', ['HEAD_JOINT0', 'HEAD_JOINT1']], ....]

    hrpsys_version = None

    # flag isKinamticsOnlyMode?
    kinematics_only_mode = False

    # public method
    def connectComps(self):
        '''!@brief
        Connect components(plugins)
        '''
        if self.rh == None or self.seq == None or self.sh == None or self.fk == None:
            print(self.configurator_name + "\033[31m connectComps : hrpsys requries rh, seq, sh and fk, please check rtcd.conf or rtcd arguments\033[0m")
            return
        # connection for reference joint angles
        tmp_controllers = self.getJointAngleControllerList()
        if len(tmp_controllers) > 0:
            connectPorts(self.sh.port("qOut"), tmp_controllers[0].port("qRef"))
            for i in range(len(tmp_controllers) - 1):
                connectPorts(tmp_controllers[i].port("q"),
                             tmp_controllers[i + 1].port("qRef"))
            if self.simulation_mode:
                if self.pdc:
                    connectPorts(tmp_controllers[-1].port("q"), self.pdc.port("angleRef"))
                else:
                    connectPorts(tmp_controllers[-1].port("q"), self.hgc.port("qIn"))
                    connectPorts(self.hgc.port("qOut"), self.rh.port("qRef"))
            else:
                connectPorts(tmp_controllers[-1].port("q"), self.rh.port("qRef"))
        else:
            if self.simulation_mode:
                if self.pdc:
                    connectPorts(self.sh.port("qOut"), self.pdc.port("angleRef"))
                else:
                    connectPorts(self.sh.port("qOut"), self.hgc.port("qIn"))
                    connectPorts(self.hgc.port("qOut"), self.rh.port("qRef"))
            else:
                connectPorts(self.sh.port("qOut"), self.rh.port("qRef"))

        # only for kinematics simulator
        if rtm.findPort(self.rh.ref, "basePoseRef"):
            self.kinematics_only_mode = True
            if self.abc:
                connectPorts(self.abc.port("basePoseOut"), self.rh.port("basePoseRef"))
            else:
                connectPorts(self.sh.port("basePoseOut"), self.rh.port("basePoseRef"))

        # connection for kf
        if self.kf:
            #   currently use first acc and rate sensors for kf
            s_acc = filter(lambda s: s.type == 'Acceleration', self.sensors)
            if (len(s_acc) > 0) and self.rh.port(s_acc[0].name) != None:  # check existence of sensor ;; currently original HRP4C.xml has different naming rule of gsensor and gyrometer
                connectPorts(self.rh.port(s_acc[0].name), self.kf.port('acc'))
            s_rate = filter(lambda s: s.type == 'RateGyro', self.sensors)
            if (len(s_rate) > 0) and self.rh.port(s_rate[0].name) != None:  # check existence of sensor ;; currently original HRP4C.xml has different naming rule of gsensor and gyrometer
                connectPorts(self.rh.port(s_rate[0].name), self.kf.port("rate"))
            connectPorts(self.rh.port("q"), self.kf.port("qCurrent"))

        # connection for rh
        if self.rh.port("servoState") != None:
            if self.te and self.el:
                connectPorts(self.rh.port("servoState"),
                             self.te.port("servoStateIn"))
                connectPorts(self.te.port("servoStateOut"),
                             self.el.port("servoStateIn"))
            elif self.el:
                connectPorts(self.rh.port("servoState"),
                             self.el.port("servoStateIn"))
            elif self.te:
                connectPorts(self.rh.port("servoState"),
                             self.te.port("servoStateIn"))

        # connection for sh, seq, fk
        connectPorts(self.rh.port("q"), [self.sh.port("currentQIn"),
                                         self.fk.port("q")])  # connection for actual joint angles
        connectPorts(self.sh.port("qOut"), self.fk.port("qRef"))
        connectPorts(self.seq.port("qRef"), self.sh.port("qIn"))
        connectPorts(self.seq.port("tqRef"), self.sh.port("tqIn"))
        connectPorts(self.seq.port("basePos"), self.sh.port("basePosIn"))
        connectPorts(self.seq.port("baseRpy"), self.sh.port("baseRpyIn"))
        connectPorts(self.seq.port("zmpRef"), self.sh.port("zmpIn"))
        if self.seq_version >= '315.2.6':
            connectPorts(self.seq.port("optionalData"), self.sh.port("optionalDataIn"))
        connectPorts(self.sh.port("basePosOut"), [self.seq.port("basePosInit"),
                                                  self.fk.port("basePosRef")])
        connectPorts(self.sh.port("baseRpyOut"), [self.seq.port("baseRpyInit"),
                                                  self.fk.port("baseRpyRef")])
        connectPorts(self.sh.port("qOut"), self.seq.port("qInit"))
        if self.seq_version >= '315.2.0':
            connectPorts(self.sh.port("zmpOut"), self.seq.port("zmpRefInit"))
        for sen in self.getForceSensorNames():
            connectPorts(self.seq.port(sen + "Ref"),
                         self.sh.port(sen + "In"))

        # connection for st
        if rtm.findPort(self.rh.ref, "lfsensor") and rtm.findPort(
                                     self.rh.ref, "rfsensor") and self.st:
            connectPorts(self.kf.port("rpy"), self.st.port("rpy"))
            connectPorts(self.sh.port("zmpOut"), self.abc.port("zmpIn"))
            connectPorts(self.sh.port("basePosOut"), self.abc.port("basePosIn"))
            connectPorts(self.sh.port("baseRpyOut"), self.abc.port("baseRpyIn"))
            connectPorts(self.sh.port("optionalDataOut"), self.abc.port("optionalData"))
            connectPorts(self.abc.port("zmpOut"), self.st.port("zmpRef"))
            connectPorts(self.abc.port("baseRpyOut"), self.st.port("baseRpyIn"))
            connectPorts(self.abc.port("basePosOut"), self.st.port("basePosIn"))
            connectPorts(self.abc.port("accRef"), self.kf.port("accRef"))
            connectPorts(self.abc.port("contactStates"), self.st.port("contactStates"))
            connectPorts(self.abc.port("controlSwingSupportTime"), self.st.port("controlSwingSupportTime"))
            connectPorts(self.rh.port("q"), self.st.port("qCurrent"))
            connectPorts(self.seq.port("qRef"), self.st.port("qRefSeq"))
            connectPorts(self.abc.port("walkingStates"), self.st.port("walkingStates"))
            connectPorts(self.abc.port("sbpCogOffset"), self.st.port("sbpCogOffset"))
            if self.es:
                connectPorts(self.st.port("emergencySignal"), self.es.port("emergencySignal"))
            connectPorts(self.st.port("emergencySignal"), self.abc.port("emergencySignal"))

        # ref force moment connection
        for sen in self.getForceSensorNames():
            if self.abc and self.st:
                connectPorts(self.abc.port(sen),
                             self.st.port(sen + "Ref"))
                connectPorts(self.abc.port("limbCOPOffset_"+sen),
                             self.st.port("limbCOPOffset_"+sen))
            if self.rfu:
                ref_force_port_from = self.rfu.port("ref_"+sen+"Out")
            elif self.es:
                ref_force_port_from = self.es.port(sen+"Out")
            else:
                ref_force_port_from = self.sh.port(sen+"Out")
            if self.ic:
                connectPorts(ref_force_port_from,
                             self.ic.port("ref_" + sen+"In"))
            if self.abc:
                connectPorts(ref_force_port_from,
                             self.abc.port("ref_" + sen))
            if self.es:
                connectPorts(self.sh.port(sen+"Out"),
                             self.es.port(sen+"In"))
                if self.rfu:
                    connectPorts(self.es.port(sen+"Out"),
                                 self.rfu.port("ref_" + sen+"In"))
            else:
                if self.rfu:
                    connectPorts(self.sh.port(sen+"Out"),
                                 self.rfu.port("ref_" + sen+"In"))

        #  actual force sensors
        if self.rmfo:
            if self.kf: # use IMU values if exists
                # connectPorts(self.kf.port("rpy"), self.ic.port("rpy"))
                connectPorts(self.kf.port("rpy"), self.rmfo.port("rpy"))
            connectPorts(self.rh.port("q"), self.rmfo.port("qCurrent"))
            for sen in filter(lambda x: x.type == "Force", self.sensors):
                connectPorts(self.rh.port(sen.name), self.rmfo.port(sen.name))
                if self.ic:
                    connectPorts(self.rmfo.port("off_" + sen.name),
                                 self.ic.port(sen.name))
                if self.rfu:
                    connectPorts(self.rmfo.port("off_" + sen.name),
                                 self.rfu.port(sen.name))
                if self.st:
                    connectPorts(self.rmfo.port("off_" + sen.name),
                                 self.st.port(sen.name))
        elif self.ic: # if the robot does not have rmfo and kf, but have ic
            for sen in filter(lambda x: x.type == "Force", self.sensors):
                connectPorts(self.rh.port(sen.name),
                             self.ic.port(sen.name))

        # connection for ic
        if self.ic:
            connectPorts(self.rh.port("q"), self.ic.port("qCurrent"))
            if self.seq_version >= '315.3.0':
                connectPorts(self.sh.port("basePosOut"), self.ic.port("basePosIn"))
                connectPorts(self.sh.port("baseRpyOut"), self.ic.port("baseRpyIn"))
        # connection for rfu
        if self.rfu:
            if self.es:
                connectPorts(self.es.port("q"), self.rfu.port("qRef"))
            if self.seq_version >= '315.3.0':
                connectPorts(self.sh.port("basePosOut"), self.rfu.port("basePosIn"))
                connectPorts(self.sh.port("baseRpyOut"), self.rfu.port("baseRpyIn"))
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
                for vfp in filter(lambda x: str.find(x, 'v') >= 0 and
                                  str.find(x, 'sensor') >= 0, self.vs.ports.keys()):
                    connectPorts(self.vs.port(vfp), self.ic.port(vfp))
        # connection for co
        if self.co:
            connectPorts(self.rh.port("q"), self.co.port("qCurrent"))
            connectPorts(self.rh.port("servoState"), self.co.port("servoStateIn"))


        # connection for gc
        if self.gc:
            connectPorts(self.rh.port("q"), self.gc.port("qCurrent"))  # other connections
            tmp_controller = self.getJointAngleControllerList()
            index = tmp_controller.index(self.gc)
            if index == 0:
                connectPorts(self.sh.port("qOut"), self.gc.port("qIn"))
            else:
                connectPorts(tmp_controller[index - 1].port("q"),
                             self.gc.port("qIn"))

        # connection for te
        if self.te:
            connectPorts(self.rh.port("q"), self.te.port("qCurrentIn"))
            connectPorts(self.sh.port("qOut"), self.te.port("qRefIn"))
            if self.tf:
                connectPorts(self.tf.port("tauOut"), self.te.port("tauIn"))
            else:
                connectPorts(self.rh.port("tau"), self.te.port("tauIn"))
            # sevoStateIn is connected above

        # connection for tl
        if self.tl:
            if self.te:
                connectPorts(self.te.port("tempOut"), self.tl.port("tempIn"))

        # connection for tc
        if self.tc:
            connectPorts(self.rh.port("q"), self.tc.port("qCurrent"))
            if self.tf:
                connectPorts(self.tf.port("tauOut"),
                             self.tc.port("tauCurrent"))
            else:
                connectPorts(self.rh.port("tau"), self.tc.port("tauCurrent"))
            if self.tl:
                connectPorts(self.tl.port("tauMax"), self.tc.port("tauMax"))

        # connection for el
        if self.el:
            connectPorts(self.rh.port("q"), self.el.port("qCurrent"))

        # connection for co
        if self.es:
            connectPorts(self.rh.port("servoState"), self.es.port("servoStateIn"))

        if self.bp:
            if self.tl:
                connectPorts(self.tl.port("beepCommand"), self.bp.port("beepCommand"))
            if self.es:
                connectPorts(self.es.port("beepCommand"), self.bp.port("beepCommand"))
            if self.el:
                connectPorts(self.el.port("beepCommand"), self.bp.port("beepCommand"))
            if self.co:
                connectPorts(self.co.port("beepCommand"), self.bp.port("beepCommand"))

    def activateComps(self):
        '''!@brief
        Activate components(plugins)
        '''
        rtcList = self.getRTCInstanceList()
        rtm.serializeComponents(rtcList)
        for r in rtcList:
            r.start()

    def deactivateComps(self):
        '''!@brief
        Deactivate components(plugins)
        '''
        rtcList = self.getRTCInstanceList()
        for r in reversed(rtcList):
            r.stop()

    def createComp(self, compName, instanceName):
        '''!@brief
        Create RTC component (plugins)

        @param instanceName str: name of instance, choose one of https://github.com/fkanehiro/hrpsys-base/tree/master/rtc
        @param comName str: name of component that to be created.
        '''
        self.ms.load(compName)
        comp = self.ms.create(compName, instanceName)
        version = comp.ref.get_component_profile().version
        print(self.configurator_name + "create Comp -> %s : %s (%s)" % (compName, comp, version))
        if comp == None:
            raise RuntimeError("Cannot create component: " + compName)
        if comp.service("service0"):
            comp_svc = narrow(comp.service("service0"), compName + "Service")
            print(self.configurator_name + "create CompSvc -> %s Service : %s" % (compName, comp_svc))
            return [comp, comp_svc, version]
        else:
            return [comp, None, version]

    def createComps(self):
        '''!@brief
        Create components(plugins) in getRTCList()
        '''
        for rn in self.getRTCList():
            try:
                rn2 = 'self.' + rn[0]
                if eval(rn2) == None:
                    create_str = "[self." + rn[0] + ", self." + rn[0] + "_svc, self." + rn[0] + "_version] = self.createComp(\"" + rn[1] + "\",\"" + rn[0] + "\")"
                    print(self.configurator_name + "  eval : " + create_str)
                    exec(create_str)
            except Exception:
                _, e, _ = sys.exc_info()
                print(self.configurator_name + '\033[31mFail to createComps ' + str(e) + '\033[0m')


    def deleteComp(self, compName):
        '''!@brief
        Delete RTC component (plugins)

        @param compName str: name of component that to be deleted
        '''
        # component must be stoppped before delete
        comp = rtm.findRTC(compName)
        comp.stop()
        return self.ms.delete(compName)

    def deleteComps(self):
        '''!@brief
        Delete components(plugins) in getRTCInstanceList()
        '''
        self.deactivateComps()
        rtcList = self.getRTCInstanceList()
        if rtcList:
            try:
                rtcList.remove(self.rh)
                for r in reversed(rtcList):
                    if r.isActive():
                        print(self.configurator_name, '\033[31m ' + r.name() + ' is staill active\033[0m')
                    self.deleteComp(r.name())
            except Exception:
                _, e, _ = sys.exc_info()
                print(self.configurator_name + '\033[31mFail to deleteComps' + str(e) + '\033[0m')

    def findComp(self, compName, instanceName, max_timeout_count=10, verbose=True):
        '''!@brief
        Find component(plugin) 
        
        @param compName str: component name
        @param instanceName str: instance name
        @max_timeout_count int: max timeout in seconds
        '''
        timeout_count = 0
        comp = rtm.findRTC(instanceName)
        version = None
        while comp == None and timeout_count < max_timeout_count:
            comp = rtm.findRTC(instanceName)
            if comp != None:
                break
            if verbose:
                print(self.configurator_name + " find Comp wait for " + instanceName)
            time.sleep(1)
            timeout_count += 1
        if comp and comp.ref:
            version = comp.ref.get_component_profile().version
        if verbose:
            print(self.configurator_name + " find Comp    : %s = %s (%s)" % (instanceName, comp, version))
        if comp == None:
            if verbose:
                print(self.configurator_name + " Cannot find component: %s (%s)" % (instanceName, compName))
            return [None, None, None]
        comp_svc_port = comp.service("service0")
        if comp_svc_port:
            comp_svc = narrow(comp_svc_port, compName + "Service")
            if verbose:
                print(self.configurator_name + " find CompSvc : %s_svc = %s"%(instanceName, comp_svc))
            return [comp, comp_svc, version]
        else:
            return [comp, None, version]

    def findComps(self, max_timeout_count = 10, verbose=True):
        '''!@brief
        Check if all components in getRTCList() are created
        '''
        for rn in self.getRTCList():
            rn2 = 'self.' + rn[0]
            if eval(rn2) == None:
                create_str = "[self." + rn[0] + ", self." + rn[0] + "_svc, self." + rn[0] + "_version] = self.findComp(\"" + rn[1] + "\",\"" + rn[0] + "\"," + str(max_timeout_count) + "," + str(verbose) + ")"
                if verbose:
                    print(self.configurator_name + create_str)
                exec(create_str)
                if eval(rn2) == None:
                    max_timeout_count = 0

    # public method to configure all RTCs to be activated on rtcd
    def getRTCList(self):
        '''!@brief
        Get list of available STABLE components
        @return list of list: List of available components. Each element consists of a list
                 of abbreviated and full names of the component.
        '''
        return [
            ['seq', "SequencePlayer"],
            ['sh', "StateHolder"],
            ['fk', "ForwardKinematics"],
            # ['tf', "TorqueFilter"],
            # ['kf', "KalmanFilter"],
            # ['vs', "VirtualForceSensor"],
            # ['rmfo', "RemoveForceSensorLinkOffset"],
            # ['ic', "ImpedanceController"],
            # ['abc', "AutoBalancer"],
            # ['st', "Stabilizer"],
            ['co', "CollisionDetector"],
            # ['tc', "TorqueController"],
            # ['te', "ThermoEstimator"],
            # ['tl', "ThermoLimiter"],
            ['el', "SoftErrorLimiter"],
            ['log', "DataLogger"]
            ]

    # public method to configure all RTCs to be activated on rtcd which includes unstable RTCs
    def getRTCListUnstable(self):
        '''!@brief
        Get list of available components including stable and unstable.

        @return list of list: List of available unstable components. Each element consists
                 of a list of abbreviated and full names of the component.
        '''
        return [
            ['seq', "SequencePlayer"],
            ['sh', "StateHolder"],
            ['fk', "ForwardKinematics"],
            ['tf', "TorqueFilter"],
            ['kf', "KalmanFilter"],
            ['vs', "VirtualForceSensor"],
            ['rmfo', "RemoveForceSensorLinkOffset"],
            ['es', "EmergencyStopper"],
            ['rfu', "ReferenceForceUpdater"],
            ['ic', "ImpedanceController"],
            ['abc', "AutoBalancer"],
            ['st', "Stabilizer"],
            ['co', "CollisionDetector"],
            ['tc', "TorqueController"],
            ['te', "ThermoEstimator"],
            ['hes', "EmergencyStopper"],
            ['el', "SoftErrorLimiter"],
            ['tl', "ThermoLimiter"],
            ['bp', "Beeper"],
            ['log', "DataLogger"]
            ]

    def getJointAngleControllerList(self):
        '''!@brief
        Get list of controller list that need to control joint angles
        '''
        controller_list = [self.es, self.ic, self.gc, self.abc, self.st, self.co,
                           self.tc, self.hes, self.el]
        return filter(lambda c: c != None, controller_list)  # only return existing controllers

    def getRTCInstanceList(self, verbose=True):
        '''!@brief
        Get list of RTC Instance
        '''
        ret = [self.rh]
        for rtc in self.getRTCList():
            r = 'self.'+rtc[0]
            try:
                if eval(r): 
                    ret.append(eval(r))
                else:
                    if verbose:
                        print(self.configurator_name + '\033[31mFail to find instance ('+str(rtc)+') for getRTCInstanceList\033[0m')
            except Exception:
                _, e, _ = sys.exc_info()
                print(self.configurator_name + '\033[31mFail to getRTCInstanceList'+str(e)+'\033[0m')
        return ret

    # private method to replace $(PROJECT_DIR)
    # PROJECT_DIR=(OpenHRP3  installed directory)/share/OpenHRP-3.1/sample/project
    # see http://www.openrtp.jp/openhrp3/3.1.0.beta/jp/install_ubuntu.html
    def parseUrl(self, url):
        if '$(PROJECT_DIR)' in url:
            path = subprocess.Popen(['pkg-config', 'openhrp3.1', '--variable=prefix'], stdout=subprocess.PIPE).communicate()[0].rstrip()
            path = os.path.join(path, 'share/OpenHRP-3.1/sample/project')
            url = url.replace('$(PROJECT_DIR)', path)
        return url

    # public method to get bodyInfo
    def getBodyInfo(self, url):
        '''!@brief
        Get bodyInfo
        '''
        import CosNaming
        obj = rtm.rootnc.resolve([CosNaming.NameComponent('ModelLoader', '')])
        mdlldr = obj._narrow(ModelLoader_idl._0_OpenHRP__POA.ModelLoader)
        url = self.parseUrl(url)
        print(self.configurator_name + "  bodyinfo URL = file://" + url)
        return mdlldr.getBodyInfo("file://" + url)

    # public method to get sensors list
    def getSensors(self, url):
        '''!@brief
        Get list of sensors

        @param url str: model file url
        '''
        if url == '':
            return []
        else:
            return sum(map(lambda x: x.sensors,
                           filter(lambda x: len(x.sensors) > 0,
                                  self.getBodyInfo(url)._get_links())), [])  # sum is for list flatten

    # public method to get sensors list
    def getForceSensorNames(self):
        '''!@brief
        Get list of force sensor names. Returns existence force sensors and virtual force sensors. self.sensors and virtual force sensors are assumed.
        '''
        ret = map (lambda x : x.name, filter(lambda x: x.type == "Force", self.sensors))
        if self.vs != None:
            ret += filter(lambda x: str.find(x, 'v') >= 0 and str.find(x, 'sensor') >= 0, self.vs.ports.keys())
        return ret

    def connectLoggerPort(self, artc, sen_name, log_name=None):
        '''!@brief
        Connect port to logger

        @param artc object: object of component that contains sen_name port
        @param sen_name str: name of port for logging
        @param log_name str: name of port in logger
        '''
        log_name = log_name if log_name else artc.name() + "_" + sen_name
        if artc and rtm.findPort(artc.ref, sen_name) != None:
            sen_type = rtm.dataTypeOfPort(artc.port(sen_name)).split("/")[1].split(":")[0]
            if rtm.findPort(self.log.ref, log_name) == None:
                print(self.configurator_name + "  setupLogger : record type = %s, name = %s to %s" % ( sen_type, sen_name, log_name))
                self.log_svc.add(sen_type, log_name)
            else:
                print(self.configurator_name + "  setupLogger : %s arleady exists in DataLogger"% sen_name)
            connectPorts(artc.port(sen_name), self.log.port(log_name))

    # public method to configure default logger data ports
    def setupLogger(self, maxLength=4000):
        '''!@brief
        Setup logging function.
        @param maxLength : max length of data from DataLogger.h #define DEFAULT_MAX_LOG_LENGTH (200*20)
                           if the robot running at 200hz (5msec) 4000 means 20 secs
        '''
        if self.log == None:
            print(self.configurator_name + "\033[31m  setupLogger : self.log is not defined, please check rtcd.conf or rtcd arguments\033[0m")
            return
        self.log_svc.maxLength(maxLength);
        #
        for pn in ['q', 'dq', 'tau']:
            self.connectLoggerPort(self.rh, pn)
        # sensor logger ports
        print(self.configurator_name + "sensor names for DataLogger")
        for sen in self.sensors:
            self.connectLoggerPort(self.rh, sen.name)
        #
        if self.kf != None:
            self.connectLoggerPort(self.kf, 'rpy')
        if self.sh != None:
            self.connectLoggerPort(self.sh, 'qOut')
            self.connectLoggerPort(self.sh, 'tqOut')
            self.connectLoggerPort(self.sh, 'basePosOut')
            self.connectLoggerPort(self.sh, 'baseRpyOut')
            self.connectLoggerPort(self.sh, 'zmpOut')
        if self.ic != None:
            self.connectLoggerPort(self.ic, 'q')
        if self.abc != None:
            self.connectLoggerPort(self.abc, 'zmpOut')
            self.connectLoggerPort(self.abc, 'baseTformOut')
            self.connectLoggerPort(self.abc, 'q')
            self.connectLoggerPort(self.abc, 'contactStates')
            self.connectLoggerPort(self.abc, 'controlSwingSupportTime')
            self.connectLoggerPort(self.abc, 'cogOut')
        if self.st != None:
            self.connectLoggerPort(self.st, 'zmp')
            self.connectLoggerPort(self.st, 'originRefZmp')
            self.connectLoggerPort(self.st, 'originRefCog')
            self.connectLoggerPort(self.st, 'originRefCogVel')
            self.connectLoggerPort(self.st, 'originNewZmp')
            self.connectLoggerPort(self.st, 'originActZmp')
            self.connectLoggerPort(self.st, 'originActCog')
            self.connectLoggerPort(self.st, 'originActCogVel')
            self.connectLoggerPort(self.st, 'allRefWrench')
            self.connectLoggerPort(self.st, 'allEEComp')
            self.connectLoggerPort(self.st, 'q')
            self.connectLoggerPort(self.st, 'actBaseRpy')
            self.connectLoggerPort(self.st, 'currentBasePos')
            self.connectLoggerPort(self.st, 'currentBaseRpy')
            self.connectLoggerPort(self.st, 'debugData')
        if self.el != None:
            self.connectLoggerPort(self.el, 'q')
        if self.rh != None:
            self.connectLoggerPort(self.rh, 'emergencySignal',
                                   'emergencySignal')
            self.connectLoggerPort(self.rh, 'servoState')
            if self.simulation_mode:
                self.connectLoggerPort(self.rh, 'WAIST')
        for sen in filter(lambda x: x.type == "Force", self.sensors):
            self.connectLoggerPort(self.sh, sen.name + "Out")
        if self.rmfo != None:
            for sen in filter(lambda x: x.type == "Force", self.sensors):
                self.connectLoggerPort(self.rmfo, "off_"+sen.name)
        if self.rfu != None:
            for sen in filter(lambda x: x.type == "Force", self.sensors):
                self.connectLoggerPort(self.rfu, "ref_"+sen.name+"Out")
        self.log_svc.clear()
        ## parallel running log process (outside from rtcd) for saving logs by emergency signal
        if self.log and (self.log_use_owned_ec or not isinstance(self.log.owned_ecs[0], OpenRTM._objref_ExtTrigExecutionContextService)):
            print(self.configurator_name + "\033[32mruning DataLogger with own Execution Context\033[0m")
            self.log.owned_ecs[0].start()
            self.log.start(self.log.owned_ecs[0])

    def waitForRTCManager(self, managerhost=nshost):
        '''!@brief
        Wait for RTC manager.

        @param managerhost str: name of host computer that manager is running
        '''
        self.ms = None
        while self.ms == None:
            time.sleep(1)
            if managerhost == "localhost":
                managerhost = socket.gethostname()
            self.ms = rtm.findRTCmanager(managerhost)
            print(self.configurator_name + "wait for RTCmanager : " + str(managerhost))

    def waitForRobotHardware(self, robotname="Robot"):
        '''!@brief
        Wait for RobotHardware is exists and activated.

        @param robotname str: name of RobotHardware component.
        '''
        self.rh = None
        timeout_count = 0
        # wait for simulator or RobotHardware setup which sometime takes a long time
        while self.rh == None and timeout_count < 10:  # <- time out limit
            if timeout_count > 0: # do not sleep initial loop
                time.sleep(1);
            self.rh = rtm.findRTC("RobotHardware0")
            if not self.rh:
                self.rh = rtm.findRTC(robotname)
            print(self.configurator_name + "wait for %s : %s ( timeout %d < 10)" % ( robotname, self.rh, timeout_count))
            if self.rh and self.rh.isActive() == None:  # just in case rh is not ready...
                self.rh = None
            timeout_count += 1
        if not self.rh:
            print(self.configurator_name + "Could not find " + robotname)
            if self.ms:
                print(self.configurator_name + "Candidates are .... " + str([x.name()  for x in self.ms.get_components()]))
            print(self.configurator_name + "Exitting.... " + robotname)
            exit(1)

        print(self.configurator_name + "findComps -> RobotHardware : %s isActive? = %s " % (self.rh,  self.rh.isActive()))

    def checkSimulationMode(self):
        '''!@brief
        Check if this is running as simulation
        '''
        # distinguish real robot from simulation by check "HG/PD controller" ( > 315.3.0)
        # distinguish real robot from simulation by using "servoState" port  (<= 315.3.0)
        self.hgc = findRTC("HGcontroller0")
        self.pdc = findRTC("PDcontroller0")
        if self.hgc or self.pdc:
            self.simulation_mode = True
        else:
            self.simulation_mode = False

        if rtm.findPort(self.rh.ref, "servoState"): # for RobotHadware <= 315.3.0, which does not have service port
            self.rh_svc = narrow(self.rh.service("service0"), "RobotHardwareService")
            self.ep_svc = narrow(self.rh.ec, "ExecutionProfileService")

        print(self.configurator_name + "simulation_mode : %s" % self.simulation_mode)

    def waitForRTCManagerAndRoboHardware(self, robotname="Robot", managerhost=nshost):
        print("\033[93m%s waitForRTCManagerAndRoboHardware has renamed to " % self.configurator_name + \
              "waitForRTCManagerAndRoboHardware: Please update your code\033[0m")
        return self.waitForRTCManagerAndRobotHardware(robotname=robotname, managerhost=managerhost)

    def waitForRTCManagerAndRobotHardware(self, robotname="Robot", managerhost=nshost):
        '''!@brief
        Wait for both RTC Manager (waitForRTCManager()) and RobotHardware (waitForRobotHardware())

        @param managerhost str: name of host computer that manager is running
        @param robotname str: name of RobotHardware component.
        '''
        self.waitForRTCManager(managerhost)
        self.waitForRobotHardware(robotname)
        self.checkSimulationMode()

    def findModelLoader(self):
        '''!@brief
        Try to find ModelLoader
        '''
        try:
            return rtm.findObject("ModelLoader")
        except:
            return None

    def waitForModelLoader(self):
        '''!@brief
        Wait for ModelLoader.
        '''
        while self.findModelLoader() == None:  # seq uses modelloader
            print(self.configurator_name + "wait for ModelLoader")
            time.sleep(3)

    def setSelfGroups(self):
        '''!@brief
        Set to the hrpsys.SequencePlayer the groups of links and joints that
        are statically defined as member variables (Groups) within this class.
        '''
        for item in self.Groups:
            self.seq_svc.addJointGroup(item[0], item[1])

    # #
    # # service interface for RTC component
    # #
    def goActual(self):
        '''!@brief
        Adjust commanded values to the angles in the physical state
        by calling StateHolder::goActual.

        This needs to be run BEFORE servos are turned on.
        '''
        self.sh_svc.goActual()

    def setJointAngle(self, jname, angle, tm):
        '''!@brief
        Set angle to the given joint.
        \verbatim
        NOTE-1: It's known that this method does not do anything after
                some group operation is done.
                TODO: at least need to warn users.
        NOTE-2: While this method does not check angle value range,
                any joints could emit position limit over error, which has not yet
                been thrown by hrpsys so that there's no way to catch on this python client. 
                Worthwhile opening an enhancement ticket at designated issue tracker.
        \endverbatim

        @param jname str: name of joint
        @param angle float: In degree.
        @param tm float: Time to complete.
        '''
        radangle = angle / 180.0 * math.pi
        return self.seq_svc.setJointAngle(jname, radangle, tm)

    def setJointAngles(self, angles, tm):
        '''!@brief
        Set all joint angles.
        \verbatim
        NOTE: While this method does not check angle value range,
              any joints could emit position limit over error, which has not yet
              been thrown by hrpsys so that there's no way to catch on this python client. 
              Worthwhile opening an enhancement ticket at designated issue tracker.
        \endverbatim
        @param angles list of float: In degree.
        @param tm float: Time to complete.
        '''
        ret = []
        for angle in angles:
            ret.append(angle / 180.0 * math.pi)
        return self.seq_svc.setJointAngles(ret, tm)

    def setJointAnglesOfGroup(self, gname, pose, tm, wait=True):
        '''!@brief
        Set the joint angles to aim. By default it waits interpolation to be
        over.

        \verbatim
        NOTE: While this method does not check angle value range,
              any joints could emit position limit over error, which has not yet
              been thrown by hrpsys so that there's no way to catch on this python client. 
              Worthwhile opening an enhancement ticket at designated issue tracker.
        \endverbatim

        @param gname str: Name of the joint group.
        @param pose list of float: list of positions and orientations
        @param tm float: Time to complete.
        @param wait bool: If true, all other subsequent commands wait until
                          the movement commanded by this method call finishes.
        '''
        angles = [x / 180.0 * math.pi for x in pose]
        ret = self.seq_svc.setJointAnglesOfGroup(gname, angles, tm)
        if wait:
            self.waitInterpolationOfGroup(gname)
        return ret

    def setJointAnglesSequence(self, angless, tms):
        '''!@brief
        Set all joint angles.
        \verbatim
        NOTE: While this method does not check angle value range,
              any joints could emit position limit over error, which has not yet
              been thrown by hrpsys so that there's no way to catch on this python client. 
              Worthwhile opening an enhancement ticket at designated issue tracker.
        \endverbatim
        @param sequence angles list of float: In degree.
        @param tm sequence of float: Time to complete, In Second
        '''
        for angles in angless:
            for i in range(len(angles)):
                angles[i] = angles[i] / 180.0 * math.pi
        return self.seq_svc.setJointAnglesSequence(angless, tms)

    def setJointAnglesSequenceOfGroup(self, gname, angless, tms):
        '''!@brief
        Set all joint angles.
        \verbatim
        NOTE: While this method does not check angle value range,
              any joints could emit position limit over error, which has not yet
              been thrown by hrpsys so that there's no way to catch on this python client. 
              Worthwhile opening an enhancement ticket at designated issue tracker.
        \endverbatim
        @param gname str: Name of the joint group.
        @param sequence angles list of float: In degree.
        @param tm sequence of float: Time to complete, In Second
        '''
        for angles in angless:
            for i in range(len(angles)):
                angles[i] = angles[i] / 180.0 * math.pi
        return self.seq_svc.setJointAnglesSequenceOfGroup(gname, angless, tms)

    def loadPattern(self, fname, tm):
        '''!@brief
        Load a pattern file that is created offline.

        Format of the pattern file:
        - example format:
        \verbatim
          t0 j0 j1 j2...jn
          t1 j0 j1 j2...jn
          :
          tn j0 j1 j2...jn
        \endverbatim
        - Delimmitted by space
        - Each line consists of an action.
        - Time between each action is defined by tn+1 - tn
          - The time taken for the 1st line is defined by the arg tm.

        @param fname str: Name of the pattern file.
        @param tm float: - The time to take for the 1st line.
        @return: List of 2 oct(string) values.
        '''
        fname = self.parseUrl(fname)
        return self.seq_svc.loadPattern(fname, tm)

    def waitInterpolation(self):
        '''!@brief
        Lets SequencePlayer wait until the movement currently happening to
        finish.
        '''
        self.seq_svc.waitInterpolation()

    def waitInterpolationOfGroup(self, gname):
        '''!@brief
        Lets SequencePlayer wait until the movement currently happening to
        finish.

        @see: SequencePlayer.waitInterpolationOfGroup
        @see: http://wiki.ros.org/joint_trajectory_action. This method
              corresponds to JointTrajectoryGoal in ROS.

        @param gname str: Name of the joint group.
        '''
        self.seq_svc.waitInterpolationOfGroup(gname)

    def setInterpolationMode(self, mode):
        '''!@brief
        set interpolation mode.
        @param i_mode_ new interpolation mode
        @return true if set successfully, false otherwise
        '''
        return self.seq_svc.setInterpolationMode(mode)

    def getJointAngles(self):
        '''!@brief
        Returns the commanded joint angle values.

        @see: HrpsysConfigurator.getJointAngles

        Note that it's not the physical state of the robot's joints, which
        can be obtained by getActualState().angle.

        @return List of float: List of angles (degree) of all joints, in the order defined
                 in the member variable 'Groups' (eg. chest, head1, head2, ..).
        '''
        return [x * 180.0 / math.pi for x in self.sh_svc.getCommand().jointRefs]

    def getCurrentPose(self, lname=None, frame_name=None):
        '''!@brief
        Returns the current physical pose of the specified joint.
        cf. getReferencePose that returns commanded value.

        eg.
        \verbatim
             IN: robot.getCurrentPose('LARM_JOINT5')
             OUT: [-0.0017702356144599085,
              0.00019034630541264752,
              -0.9999984150158207,
              0.32556275164378523,
              0.00012155879975329215,
              0.9999999745367515,
               0.0001901314142046251,
               0.18236394191140365,
               0.9999984257434246,
               -0.00012122202968358842,
               -0.001770258707652326,
               0.07462472659364472,
               0.0,
               0.0,
               0.0,
               1.0]
        \endverbatim

        @type lname: str
        @param lname: Name of the link.
        @param frame_name str: set reference frame name (from 315.2.5)
        @rtype: list of float
        @return: Rotational matrix and the position of the given joint in
                 1-dimensional list, that is:
        \verbatim
                 [a11, a12, a13, x,
                  a21, a22, a23, y,
                  a31, a32, a33, z,
                   0,   0,   0,  1]
        \endverbatim
        '''
        if not lname:
            for item in self.Groups:
                eef_name = item[1][-1]
                print("{}: {}".format(eef_name, self.getCurrentPose(eef_name)))
            raise RuntimeError("need to specify joint name")
        if frame_name:
            lname = lname + ':' + frame_name
        if self.fk_version < '315.2.5' and ':' in lname:
            raise RuntimeError('frame_name ('+lname+') is not supported')
        pose = self.fk_svc.getCurrentPose(lname)
        if not pose[0]:
            raise RuntimeError("Could not find reference : " + lname)
        return pose[1].data

    def getCurrentPosition(self, lname=None, frame_name=None):
        '''!@brief
        Returns the current physical position of the specified joint.
        cf. getReferencePosition that returns commanded value.

        eg.
        \verbatim
            robot.getCurrentPosition('LARM_JOINT5')
            [0.325, 0.182, 0.074]
        \endverbatim

        @type lname: str
        @param lname: Name of the link.
        @param frame_name str: set reference frame name (from 315.2.5)
        @rtype: list of float
        @return: List of x, y, z positions about the specified joint.
        '''
        if not lname:
            for item in self.Groups:
                eef_name = item[1][-1]
                print("{}: {}".format(eef_name, self.getCurrentPosition(eef_name)))
            raise RuntimeError("need to specify joint name")
        pose = self.getCurrentPose(lname, frame_name)
        return [pose[3], pose[7], pose[11]]

    def getCurrentRotation(self, lname, frame_name=None):
        '''!@brief
        Returns the current physical rotation of the specified joint.
        cf. getReferenceRotation that returns commanded value.

        @type lname: str
        @param lname: Name of the link.
        @param frame_name str: set reference frame name (from 315.2.5)
        @rtype: list of float
        @return: Rotational matrix of the given joint in 2-dimensional list,
                 that is:
        \verbatim
                 [[a11, a12, a13],
                  [a21, a22, a23],
                  [a31, a32, a33]]
        \endverbatim
        '''
        if not lname:
            for item in self.Groups:
                eef_name = item[1][-1]
                print("{}: {}".format(eef_name, self.getCurrentRotation(eef_name)))
            raise RuntimeError("need to specify joint name")
        pose = self.getCurrentPose(lname, frame_name)
        return [pose[0:3], pose[4:7], pose[8:11]]

    def getCurrentRPY(self, lname, frame_name=None):
        '''!@brief
        Returns the current physical rotation in RPY of the specified joint.
        cf. getReferenceRPY that returns commanded value.

        @type lname: str
        @param lname: Name of the link.
        @param frame_name str: set reference frame name (from 315.2.5)
        @rtype: list of float
        @return: List of orientation in rpy form about the specified joint.
        '''
        if not lname:
            for item in self.Groups:
                eef_name = item[1][-1]
                print("{}: {}".format(eef_name, self.getCurrentRPY(eef_name)))
            raise RuntimeError("need to specify joint name")
        return euler_from_matrix(self.getCurrentRotation(lname), 'sxyz')

    def getReferencePose(self, lname, frame_name=None):
        '''!@brief
        Returns the current commanded pose of the specified joint.
        cf. getCurrentPose that returns physical pose.

        @type lname: str
        @param lname: Name of the link.
        @param frame_name str: set reference frame name (from 315.2.5)
        @rtype: list of float
        @return: Rotational matrix and the position of the given joint in
                 1-dimensional list, that is:
        \verbatim
                 [a11, a12, a13, x,
                  a21, a22, a23, y,
                  a31, a32, a33, z,
                   0,   0,   0,  1]
        \endverbatim
        '''
        if not lname:
            for item in self.Groups:
                eef_name = item[1][-1]
                print("{}: {}".format(eef_name, self.getReferencePose(eef_name)))
            raise RuntimeError("need to specify joint name")
        if frame_name:
            lname = lname + ':' + frame_name
        if self.fk_version < '315.2.5' and ':' in lname:
            raise RuntimeError('frame_name ('+lname+') is not supported')
        pose = self.fk_svc.getReferencePose(lname)
        if not pose[0]:
            raise RuntimeError("Could not find reference : " + lname)
        return pose[1].data

    def getReferencePosition(self, lname, frame_name=None):
        '''!@brief
        Returns the current commanded position of the specified joint.
        cf. getCurrentPosition that returns physical value.

        @type lname: str
        @param lname: Name of the link.
        @param frame_name str: set reference frame name (from 315.2.5)
        @rtype: list of float
        @return: List of angles (degree) of all joints, in the order defined
                 in the member variable 'Groups' (eg. chest, head1, head2, ..).
        '''
        if not lname:
            for item in self.Groups:
                eef_name = item[1][-1]
                print("{}: {}".format(eef_name, self.getReferencePosition(eef_name)))
            raise RuntimeError("need to specify joint name")
        pose = self.getReferencePose(lname, frame_name)
        return [pose[3], pose[7], pose[11]]

    def getReferenceRotation(self, lname, frame_name=None):
        '''!@brief
        Returns the current commanded rotation of the specified joint.
        cf. getCurrentRotation that returns physical value.

        @type lname: str
        @param lname: Name of the link.
        @param frame_name str: set reference frame name (from 315.2.5)
        @rtype: list of float
        @return: Rotational matrix of the given joint in 2-dimensional list,
                 that is:
        \verbatim
                 [[a11, a12, a13],
                  [a21, a22, a23],
                  [a31, a32, a33]]
        \endverbatim
        '''
        if not lname:
            for item in self.Groups:
                eef_name = item[1][-1]
                print("{}: {}".format(eef_name, self.getReferencePotation(eef_name)))
            raise RuntimeError("need to specify joint name")
        pose = self.getReferencePose(lname, frame_name)
        return [pose[0:3], pose[4:7], pose[8:11]]

    def getReferenceRPY(self, lname, frame_name=None):
        '''!@brief
        Returns the current commanded rotation in RPY of the specified joint.
        cf. getCurrentRPY that returns physical value.

        @type lname: str
        @param lname: Name of the link.
        @param frame_name str: set reference frame name (from 315.2.5)
        @rtype: list of float
        @return: List of orientation in rpy form about the specified joint.
        '''
        if not lname:
            for item in self.Groups:
                eef_name = item[1][-1]
                print("{}: {}".format(eef_name, self.getReferenceRPY(eef_name)))
            raise RuntimeError("need to specify joint name")
        return euler_from_matrix(self.getReferenceRotation(lname, frame_name), 'sxyz')

    def setTargetPose(self, gname, pos, rpy, tm, frame_name=None):
        '''!@brief
        Move the end-effector to the given absolute pose.
        All d* arguments are in meter.

        @param gname str: Name of the joint group.
        @param pos list of float: In meter.
        @param rpy list of float: In radian.
        @param tm float: Second to complete the command.
        @param frame_name str: Name of the frame that this particular command
                           references to.
        @return bool: False if unreachable.
        '''
        print(gname, frame_name, pos, rpy, tm)
        if frame_name:
            gname = gname + ':' + frame_name
        result = self.seq_svc.setTargetPose(gname, pos, rpy, tm)
        if not result:
            print("setTargetPose failed. Maybe SequencePlayer failed to solve IK.\n"
                   + "Currently, returning IK result error\n"
                   + "(like the one in https://github.com/start-jsk/rtmros_hironx/issues/103)"
                   + " is not implemented. Patch is welcomed.")
        return result

    def setTargetPoseRelative(self, gname, eename, dx=0, dy=0, dz=0,
dr=0, dp=0, dw=0, tm=10, wait=True):
        '''!@brief
        Move the end-effector's location relative to its current pose.

        For d*, distance arguments are in meter while rotations are in degree.

        Example usage: The following moves RARM_JOINT5 joint 0.1mm forward
                       within 0.1sec.
        \verbatim
            robot.setTargetPoseRelative('rarm', 'RARM_JOINT5', dx=0.0001, tm=0.1)
        \endverbatim
        @param gname str: Name of the joint group that is to be manipulated.
        @param eename str: Name of the joint that the manipulated joint group references to.
        @param dx float: In meter.
        @param dy float: In meter.
        @param dz float: In meter.
        @param dr float: In radian.
        @param dp float: In radian.
        @param dw float: In radian.
        @param tm float: Second to complete the command.
        @param wait bool: If true, all other subsequent commands wait until
                          the movement commanded by this method call finishes.
        @return bool: False if unreachable.
        '''
        self.waitInterpolationOfGroup(gname)
        # curPose = self.getCurrentPose(eename)
        posRef = None
        rpyRef = None
        tds = self.getCurrentPose(eename)
        if tds:
            posRef = numpy.array([tds[3], tds[7], tds[11]])
            matRef = numpy.array([tds[0:3], tds[4:7], tds[8:11]])
            posRef += [dx, dy, dz]
            matRef = matRef.dot(numpy.array(euler_matrix(dr, dp, dw)[:3, :3])) 
            rpyRef = euler_from_matrix(matRef)
            print(posRef, rpyRef)
            ret = self.setTargetPose(gname, list(posRef), list(rpyRef), tm)
            if ret and wait:
                self.waitInterpolationOfGroup(gname)
            return ret
        return False

    def clear(self):
        '''!@brief
        Clears the Sequencer's current operation. Works for joint groups too.
        @see HrpsysConfigurator.clear

        Discussed in https://github.com/fkanehiro/hrpsys-base/issues/158
        Examples is found in a unit test: https://github.com/start-jsk/rtmros_hironx/blob/bb0672be3e03e5366e03fe50520e215302b8419f/hironx_ros_bridge/test/test_hironx.py#L293
        '''
        self.seq_svc.clear()

    def clearOfGroup(self, gname, tm=0.0):
        '''!@brief
        Clears the Sequencer's current operation for joint groups.
        '''
        self.seq_svc.clearOfGroup(gname, tm)

    def saveLog(self, fname='sample'):
        '''!@brief
        Save log to the given file name
        
        @param fname str: name of the file
        '''
        self.log_svc.save(fname)
        print(self.configurator_name + "saved data to " + fname)

    def clearLog(self):
        '''!@brief
        Clear logger's buffer
        '''
        self.log_svc.clear()

    def setMaxLogLength(self, length):
        '''!@brief
        Set logger's buffer
        @param length int: length of log, if the system runs at 500hz and you want to store 2min, set 2*60*500.
        '''
        self.log_svc.maxLength(length)

    def lengthDigitalInput(self):
        '''!@brief
        Returns the length of digital input port
        '''
        return self.rh_svc.lengthDigitalInput()

    def lengthDigitalOutput(self):
        '''!@brief
        Returns the length of digital output port
        '''
        return self.rh_svc.lengthDigitalOutput()

    def writeDigitalOutput(self, dout):
        '''!@brief
        Using writeDigitalOutputWithMask is recommended for the less data
        transport.

        @param dout list of int: List of bits, length of 32 bits where elements are
                     0 or 1.

                     What each element stands for depends on how
                     the robot's imlemented. Consult the hardware manual.
        @return bool: RobotHardware.writeDigitalOutput returns True if writable. False otherwise.
        '''
        if self.simulation_mode:
            return True
        doutBitLength = self.lengthDigitalOutput() * 8
        if len(dout) < doutBitLength:
            for i in range(doutBitLength - len(dout)):
                dout.append(0)
        outStr = ''
        for i in range(0, len(dout), 8):
            oneChar = 0
            for j in range(8):
                if dout[i + j]:
                    oneChar += 1 << j
            outStr = outStr + chr(oneChar)

        # octet sequences are mapped to strings in omniorbpy
        return self.rh_svc.writeDigitalOutput(outStr)

    def writeDigitalOutputWithMask(self, dout, mask):
        '''!@brief
        Both dout and mask are lists with length of 32. Only the bit in dout
        that corresponds to the bits in mask that are flagged as 1 will be
        evaluated.

        Example:
        \verbatim
         Case-1. Only 18th bit will be evaluated as 1.
          dout [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
          mask [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

         Case-2. Only 18th bit will be evaluated as 0.
          dout [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
          mask [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

         Case-3. None will be evaluated since there's no flagged bit in mask.
          dout [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
          mask [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        \endverbatim
        @param dout list of int: List of bits, length of 32 bits where elements are
                     0 or 1.
        @param mask list of int: List of masking bits, length of 32 bits where elements are
                     0 or 1.
        @return bool: RobotHardware.writeDigitalOutput returns True if writable. False otherwise.
        '''
        if self.simulation_mode:
            return True
        doutBitLength = self.lengthDigitalOutput() * 8
        if len(dout) < doutBitLength and \
               len(mask) < doutBitLength and \
               len(dout) == len(mask):
            for i in range(doutBitLength - len(dout)):
                dout.append(0)
                mask.append(0)
        outStr = ''
        outMsk = ''
        for i in range(0, len(dout), 8):
            oneChar = 0
            oneMask = 0
            for j in range(8):
                if dout[i + j]:
                    oneChar += 1 << j
                if mask[i + j]:
                    oneMask += 1 << j
            outStr = outStr + chr(oneChar)
            outMsk = outMsk + chr(oneMask)

        # octet sequences are mapped to strings in omniorbpy
        return self.rh_svc.writeDigitalOutputWithMask(outStr, outMsk)

    def readDigitalInput(self):
        '''!@brief
        Read data from Digital Input
        @see: HrpsysConfigurator.readDigitalInput

        Digital input consits of 14 bits. The last 2 bits are lacking
        and compensated, so that the last 4 bits are 0x4 instead of 0x1.
        @author Hajime Saito (\@emijah)
        @return list of int: List of the values (2 octtets) in digital input register. Range: 0 or 1.

        #TODO: Catch AttributeError that occurs when RobotHardware not found.
        #      Typically with simulator, erro msg might look like this;
        #      'NoneType' object has no attribute 'readDigitalInput'
        '''
        if self.simulation_mode:
            return []
        ret, din = self.rh_svc.readDigitalInput()
        retList = []
        for item in din:
            for i in range(8):
                if (ord(item) >> i) & 1:
                    retList.append(1)
                else:
                    retList.append(0)
        return retList

    def readDigitalOutput(self):
        '''!@brief
        Read data from Digital Output

        Digital input consits of 14 bits. The last 2 bits are lacking
        and compensated, so that the last 4 bits are 0x4 instead of 0x1.

        #TODO: Catch AttributeError that occurs when RobotHardware not found.
        #      Typically with simulator, erro msg might look like this;
        #      'NoneType' object has no attribute 'readDigitaloutput'

        @author Hajime Saito (\@emijah)
        @return list of int: List of the values in digital input register. Range: 0 or 1.
        '''
        ret, dout = self.rh_svc.readDigitalOutput()
        retList = []
        for item in dout:
            for i in range(8):
                if (ord(item) >> i) & 1:
                    retList.append(1)
                else:
                    retList.append(0)
        return retList

    def getActualState(self):
        '''!@brief
        Get actual states of the robot that includes current reference joint angles and joint torques.
        @return: This returns actual states of the robot, which is defined in
        RobotHardware.idl
        (https://github.com/fkanehiro/hrpsys-base/blob/master/idl/RobotHardwareService.idl#L33)
        \verbatim
            /**
             * @brief status of the robot
             */
            struct RobotState
            {
              DblSequence               angle;  ///< current joint angles[rad]
              DblSequence               command;///< reference joint angles[rad]
              DblSequence               torque; ///< joint torques[Nm]
              /**
               * @brief servo statuses(32bit+extra states)
               *
               * 0: calib status ( 1 => done )\n
               * 1: servo status ( 1 => on )\n
               * 2: power status ( 1 => supplied )\n
               * 3-18: servo alarms (see @ref iob.h)\n
               * 19-23: unused
                       * 24-31: driver temperature (deg)
               */
              LongSequenceSequence              servoState;
              sequence<DblSequence6>    force;    ///< forces[N] and torques[Nm]
              sequence<DblSequence3>    rateGyro; ///< angular velocities[rad/s]
              sequence<DblSequence3>    accel;    ///< accelerations[m/(s^2)]
              double                    voltage;  ///< voltage of power supply[V]
              double                    current;  ///< current[A]
            };
        \endverbatim
        '''
        return self.rh_svc.getStatus()

    def isCalibDone(self):
        '''!@brief
        Check whether joints have been calibrated.
        @return bool: True if all joints are calibrated
        '''
        if self.simulation_mode:
            return True
        else:
            rstt = self.rh_svc.getStatus()
            for item in rstt.servoState:
                if not item[0] & 1:
                    return False
        return True

    def isServoOn(self, jname='any'):
        '''!@brief
        Check whether servo control has been turned on.
        @param jname str: Name of a link (that can be obtained by "hiro.Groups"
                      as lists of groups).
                      When jname = 'all' or 'any' => If all joints are servoOn, return True, otherwise, return False.
                      When jname = 'some' => If some joint is servoOn, return True, otherwise return False.
        @return bool: True if servo is on
        '''
        if self.simulation_mode:
            return True
        else:
            s_s = self.getActualState().servoState
            if jname.lower() == 'any' or jname.lower() == 'all':
                for s in s_s:
                    # print(self.configurator_name, 's = ', s)
                    if (s[0] & 2) == 0:
                        return False
                return True
            elif jname.lower() == 'some':
                for s in s_s:
                    # print(self.configurator_name, 's = ', s)
                    if (s[0] & 2) != 0:
                        return True
                return False
            else:
                jid = eval('self.' + jname)
                print(self.configurator_name + s_s[jid])
                if s_s[jid][0] & 1 == 0:
                    return False
                else:
                    return True

    def flat2Groups(self, flatList):
        '''!@brief
        Convert list of angles into list of joint list for each groups.
        @param flatList list: single dimension list with its length of 15
        @return list of list: 2-dimensional list of Groups.
        '''
        retList = []
        index = 0
        for group in self.Groups:
            joint_num = len(group[1])
            retList.append(flatList[index: index + joint_num])
            index += joint_num
        return retList

    def servoOn(self, jname='all', destroy=1, tm=3):
        '''!@brief
        Turn on servos.
        Joints need to be calibrated (otherwise error returns).

        @param jname str: The value 'all' works iteratively for all servos.
        @param destroy int: Not used.
        @param tm float: Second to complete.
        @return int: 1 or -1 indicating success or failure, respectively.
        '''
        # check joints are calibrated
        if not self.isCalibDone():
            waitInputConfirm('!! Calibrate Encoders with checkEncoders first !!\n\n')
            return -1

        # check servo state
        if self.isServoOn():
            return 1

        # check jname is acceptable
        if jname == '':
            jname = 'all'

        try:
            r = waitInputConfirm(\
                '!! Robot Motion Warning (SERVO_ON) !!\n\n'
                'Confirm RELAY switched ON\n'
                'Push [OK] to switch servo ON(%s).' % (jname))
            if not r:
                print(self.configurator_name, 'servo on: canceled')
                return 0
        except:  # ths needs to change
            self.rh_svc.power('all', SWITCH_OFF)
            raise

        try:
            self.goActual()
            time.sleep(0.1)
            self.rh_svc.servo(jname, SWITCH_ON)
            time.sleep(2)
            if not self.isServoOn(jname):
                print(self.configurator_name + 'servo on failed.')
                raise
        except:
            print(self.configurator_name + 'exception occured')

        return 1

    def servoOff(self, jname='all', wait=True):
        '''!@brief
        Turn off servos.
        @param jname str: The value 'all' works iteratively for all servos.
        @param wait bool: Wait for user's confirmation via GUI
        @return int: 1 = all arm servo off. 2 = all servo on arms and hands off.
                -1 = Something wrong happened.
        '''
        # do nothing for simulation
        if self.simulation_mode:
            print(self.configurator_name + 'omit servo off')
            return 0

        print('Turn off Hand Servo')
        if self.sc_svc:
            self.sc_svc.servoOff()
        # if the servos aren't on switch power off
        if not self.isServoOn(jname):
            if jname.lower() == 'all':
                self.rh_svc.power('all', SWITCH_OFF)
            return 1

        # if jname is not set properly set to all -> is this safe?
        if jname == '':
            jname = 'all'

        if wait:
            r = waitInputConfirm(
                '!! Robot Motion Warning (Servo OFF)!!\n\n'
                'Push [OK] to servo OFF (%s).' % (jname))  # :
            if not r:
                print(self.configurator_name, 'servo off: canceled')
                return 2
        try:
            self.rh_svc.servo('all', SWITCH_OFF)
            time.sleep(0.2)
            if jname == 'all':
                self.rh_svc.power('all', SWITCH_OFF)

            # turn off hand motors
            print('Turn off Hand Servo')
            if self.sc_svc:
                self.sc_svc.servoOff()

            return 2
        except:
            print(self.configurator_name + 'servo off: communication error')
            return -1

    def checkEncoders(self, jname='all', option=''):
        '''!@brief
        Run the encoder checking sequence for specified joints,
        run goActual and turn on servos.

        @param jname str: The value 'all' works iteratively for all servos.
        @param option str: Possible values are follows (w/o double quote):\
                        "-overwrite": Overwrite calibration value.
        '''
        if self.isServoOn():
            waitInputConfirm('Servo must be off for calibration')
            return
        # do not check encoders twice
        elif self.isCalibDone():
            waitInputConfirm('System has been calibrated')
            return

        self.rh_svc.power('all', SWITCH_ON)
        msg = '!! Robot Motion Warning !!\n'\
              'Turn Relay ON.\n'\
              'Then Push [OK] to '
        if option == '-overwrite':
            msg = msg + 'calibrate(OVERWRITE MODE) '
        else:
            msg = msg + 'check '

        if jname == 'all':
            msg = msg + 'the Encoders of all.'
        else:
            msg = msg + 'the Encoder of the Joint "' + jname + '".'

        try:
            waitInputConfirm(msg)
        except:
            print("If you're connecting to the robot from remote, " + \
                  "make sure tunnel X (eg. -X option with ssh).")
            self.rh_svc.power('all', SWITCH_OFF)
            return 0

        print(self.configurator_name + 'calib-joint ' + jname + ' ' + option)
        self.rh_svc.initializeJointAngle(jname, option)
        print(self.configurator_name + 'done')
        self.rh_svc.power('all', SWITCH_OFF)
        self.goActual()
        time.sleep(0.1)
        self.rh_svc.servo(jname, SWITCH_ON)

        # turn on hand motors
        print('Turn on Hand Servo')
        if self.sc_svc:
            self.sc_svc.servoOn()

    def removeForceSensorOffset(self):
        '''!@brief
        remove force sensor offset
        '''
        self.rh_svc.removeForceSensorOffset()

    def playPattern(self, jointangles, rpy, zmp, tm):
        '''!@brief
        Play motion pattern using a given trajectory that is represented by 
        a list of joint angles, rpy, zmp and time.

        @param jointangles list of list of float: 
                           The whole list represents a trajectory. Each element
                           of the 1st degree in the list consists of the joint
                           angles.
        @param rpy list of float: Orientation in rpy.
        @param zmp list of float: TODO: description
        @param tm float: Time to complete the task.
        @return bool:
        '''
        return self.seq_svc.playPattern(jointangles, rpy, zmp, tm)

    def playPatternOfGroup(self, gname, jointangles, tm):
        '''!@brief
        Play motion pattern using a given trajectory that is represented by 
        a list of joint angles.

        @param gname str: Name of the joint group.
        @param jointangles list of list of float: 
                           The whole list represents a trajectory. Each element
                           of the 1st degree in the list consists of the joint
                           angles. To illustrate:

                           [[a0-0, a0-1,...,a0-n], # a)ngle. 1st path in trajectory
                            [a1-0, a1-1,...,a1-n], # 2nd path in the trajectory.
                            :
                            [am-0, am-1,...,am-n]]  # mth path in the trajectory
        @param tm float: Time to complete the task.
        @return bool:
        '''
        return self.seq_svc.playPatternOfGroup(gname, jointangles, tm)

    def setSensorCalibrationJointAngles(self):
        '''!@brief
        Set joint angles for sensor calibration.
        Please override joint angles to avoid self collision.
        '''
        self.setJointAngles([0]*len(self.rh_svc.getStatus().servoState), 4.0)
        self.waitInterpolation()

    def calibrateInertiaSensor(self):
        '''!@brief
        Calibrate inertia sensor
        '''
        self.rh_svc.calibrateInertiaSensor()

    def calibrateInertiaSensorWithDialog(self):
        '''!@brief
        Calibrate inertia sensor with dialog and setting calibration pose
        '''
        r = waitInputConfirm (
                        '!! Robot Motion Warning (Move to Sensor Calibration Pose)!!\n\n'
                        'Push [OK] to move to sensor calibration pose.')
        if not r: return False
        self.setSensorCalibrationJointAngles()
        r = waitInputConfirm (
                        '!! Put the robot down!!\n\n'
                        'Push [OK] to the next step.')
        if not r: return False
        self.calibrateInertiaSensor()
        r = waitInputConfirm (
                        '!! Put the robot up!!\n\n'
                        'Push [OK] to the next step.')
        if not r: return False
        return True

    # #
    # # service interface for Unstable RTC component
    # #
    def startAutoBalancer(self, limbs=None):
        '''!@brief
        Start AutoBalancer mode
        @param limbs list of end-effector name to control.
        If Groups has rarm and larm, rleg, lleg, rarm, larm by default.
        If Groups is not defined or Groups does not have rarm and larm, rleg and lleg by default.
        '''
        if limbs==None:
            if self.Groups != None and "rarm" in map (lambda x : x[0], self.Groups) and "larm" in map (lambda x : x[0], self.Groups):
                limbs=["rleg", "lleg", "rarm", "larm"]
            else:
                limbs=["rleg", "lleg"]
        self.abc_svc.startAutoBalancer(limbs)

    def stopAutoBalancer(self):
        '''!@brief
        Stop AutoBalancer mode
        '''
        self.abc_svc.stopAutoBalancer()

    def startStabilizer(self):
        '''!@brief
        Start Stabilzier mode
        '''
        self.st_svc.startStabilizer()

    def stopStabilizer(self):
        '''!@brief
        Stop Stabilzier mode
        '''
        self.st_svc.stopStabilizer()

    def startImpedance_315_4(self, arm,
                       M_p = 100.0,
                       D_p = 100.0,
                       K_p = 100.0,
                       M_r = 100.0,
                       D_r = 2000.0,
                       K_r = 2000.0,
                       force_gain = [1, 1, 1],
                       moment_gain = [0, 0, 0],
                       sr_gain = 1.0,
                       avoid_gain = 0.0,
                       reference_gain = 0.0,
                       manipulability_limit = 0.1,
                       controller_mode = None,
                       ik_optional_weight_vector = None):
        '''!@brief
        start impedance mode

        @type arm: str name of artm to be controlled, this must be initialized using setSelfGroups()
        '''
        r, p = self.ic_svc.getImpedanceControllerParam(arm)
        if not r:
            print('{}, Failt to getImpedanceControllerParam({})'.format(self.configurator_name, arm))
            return False
        if M_p != None: p.M_p = M_p
        if D_p != None: p.M_p = D_p
        if K_p != None: p.M_p = K_p
        if M_r != None: p.M_r = M_r
        if D_r != None: p.M_r = D_r
        if K_r != None: p.M_r = K_r
        if force_gain != None: p.force_gain = force_gain
        if moment_gain != None: p.moment_gain = moment_gain
        if sr_gain != None: p.sr_gain = sr_gain
        if avoid_gain != None: p.avoid_gain = avoid_gain
        if reference_gain != None: p.reference_gain = reference_gain
        if manipulability_limit != None: p.manipulability_limit = manipulability_limit
        if controller_mode != None: p.controller_mode = controller_mode
        if ik_optional_weight_vector != None: p.ik_optional_weight_vector = ik_optional_weight_vector
        self.ic_svc.setImpedanceControllerParam(arm, p)
        return self.ic_svc.startImpedanceController(arm)

    def stopImpedance_315_4(self, arm):
        '''!@brief
        stop impedance mode
        '''
        return self.ic_svc.stopImpedanceController(arm)

    def startImpedance(self, arm, **kwargs):
        if self.hrpsys_version and self.hrpsys_version < '315.2.0':
            print(self.configurator_name + '\033[31mstartImpedance: Try to connect unsupported RTC' + str(self.hrpsys_version) + '\033[0m')
        else:
            self.startImpedance_315_4(arm, **kwargs)

    def stopImpedance(self, arm):
        if self.hrpsys_version and self.hrpsys_version < '315.2.0':
            print(self.configurator_name + '\033[31mstopImpedance: Try to connect unsupported RTC' + str(self.hrpsys_version) + '\033[0m')
        else:
            self.stopImpedance_315_4(arm)

    def startDefaultUnstableControllers (self, ic_limbs=["rarm", "larm"], abc_limbs=None):
        '''!@brief
        Start default unstable RTCs controller mode.
        Currently Stabilzier, AutoBalancer, and ImpedanceController are started.
        '''
        for limb in ic_limbs:
            self.ic_svc.startImpedanceControllerNoWait(limb)
        if abc_limbs==None:
            if self.Groups != None and "rarm" in map (lambda x : x[0], self.Groups) and "larm" in map (lambda x : x[0], self.Groups):
                abc_limbs=["rleg", "lleg", "rarm", "larm"]
            else:
                abc_limbs=["rleg", "lleg"]
        self.startAutoBalancer(abc_limbs)
        self.startStabilizer()
        for limb in ic_limbs:
            self.ic_svc.waitImpedanceControllerTransition(limb)

    def stopDefaultUnstableControllers (self, ic_limbs=["rarm", "larm"]):
        '''!@brief
        Stop default unstable RTCs controller mode.
        Currently Stabilzier, AutoBalancer, and ImpedanceController are stopped.
        '''
        self.stopStabilizer()
        for limb in ic_limbs:
            self.ic_svc.stopImpedanceControllerNoWait(limb)
        self.stopAutoBalancer()
        for limb in ic_limbs:
            self.ic_svc.waitImpedanceControllerTransition(limb)

    def setFootSteps(self, footstep, overwrite_fs_idx = 0):
        '''!@brief
        setFootSteps
        @param footstep : FootstepSequence.
        @param overwrite_fs_idx : Index to be overwritten. overwrite_fs_idx is used only in walking.
        '''
        self.abc_svc.setFootSteps(footstep, overwrite_fs_idx)

    def setFootStepsWithParam(self, footstep, stepparams, overwrite_fs_idx = 0):
        '''!@brief
        setFootSteps
        @param footstep : FootstepSequence.
        @param stepparams : StepParamSeuqnce.
        @param overwrite_fs_idx : Index to be overwritten. overwrite_fs_idx is used only in walking.
        '''
        self.abc_svc.setFootStepsWithParam(footstep, stepparams, overwrite_fs_idx)

    # ##
    # ## initialize
    # ##

    def init(self, robotname="Robot", url=""):
        '''!@brief
        Calls init from its superclass, which tries to connect RTCManager,
        looks for ModelLoader, and starts necessary RTC components. Also runs
        config, logger.
        Also internally calls setSelfGroups().

        @type robotname: str
        @type url: str
        '''
        print(self.configurator_name + "waiting ModelLoader")
        self.waitForModelLoader()
        print(self.configurator_name + "start hrpsys")

        print(self.configurator_name + "finding RTCManager and RobotHardware")
        self.waitForRTCManagerAndRobotHardware(robotname)
        self.sensors = self.getSensors(url)

        print(self.configurator_name + "creating components")
        self.createComps()

        print(self.configurator_name + "connecting components")
        self.connectComps()

        print(self.configurator_name + "activating components")
        self.activateComps()

        print(self.configurator_name + "setup logger")
        self.setupLogger()

        print(self.configurator_name + "setup joint groups")
        self.setSelfGroups()

        print(self.configurator_name + '\033[32minitialized successfully\033[0m')

        # set hrpsys_version
        try:
            self.hrpsys_version = self.fk.ref.get_component_profile().version
        except:
            print(self.configurator_name + '\033[34mCould not get hrpsys_version\033[0m')

            pass

    def __init__(self, cname="[hrpsys.py] "):
        initCORBA()
        self.configurator_name = cname


if __name__ == '__main__':
    hcf = HrpsysConfigurator()
    if len(sys.argv) > 2:
        hcf.init(sys.argv[1], sys.argv[2])
    elif len(sys.argv) > 1:
        hcf.init(sys.argv[1])
    else:
        hcf.init()

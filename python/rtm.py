from omniORB import CORBA, any, cdrUnmarshal, cdrMarshal
import CosNaming

import OpenRTM_aist
import RTC, OpenRTM, SDOPackage, RTM
from OpenRTM import CdrData, OutPortCdr, InPortCdr
from RTC import *

import sys
import string, math, socket
import os
import time
import re

##
# \brief root naming context
#
rootnc = None
nshost = None
nsport = None

##
# \brief wrapper class of RT component
#
class RTcomponent:
    ##
    # \brief constructor
    # \param self this object
    # \param ref IOR of RT component
    def __init__(self, ref):
        self.ref = ref
        self.owned_ecs = ref.get_owned_contexts()
        self.ec = self.owned_ecs[0]
        self.ports = {}
        ports = self.ref.get_ports()
        for p in ports:
            prof = p.get_port_profile()
            name = prof.name.split('.')[1]
            self.ports[name] = p

    ##
    # \brief get IOR of port
    # \param self this object
    # \param name name of the port
    # \return IOR of the port
    def port(self, name):
        try:
            p = self.ports[unicode(name)]
        except KeyError:
            p = findPort(self.ref, name)
            self.ports[unicode(name)] = p
        return p

    ##
    # \brief get IOR of the service
    # \param self this object
    # \param instance_name instance name of the service
    # \param type_name type name of hte service
    # \param port_name port name which provides the service
    # \return IOR of the service
    def service(self, instance_name, type_name="", port_name=""):
        return findService(self, port_name, type_name, instance_name)

    ##
    # \brief update default configuration set
    # \param self this object
    # \param nvlist list of pairs of name and value
    # \return True if all values are set correctly, False otherwise
    def setConfiguration(self, nvlist):
        return setConfiguration(self.ref, nvlist)

    ##
    # \brief update value of the default configuration set
    # \param self this object
    # \param name name of the property
    # \param value new value of the property
    # \return True if all values are set correctly, False otherwise
    def setProperty(self, name, value):
        return self.setConfiguration([[name, value]])

    ##
    # \brief get value of the property in the default configuration set
    # \param self this object
    # \param name name of the property
    # \return value of the property or None if the property is not found
    def getProperty(self, name):
        cfg = self.ref.get_configuration()
        cfgsets = cfg.get_configuration_sets()
        if len(cfgsets) == 0:
            print("configuration set is not found")
            return None
        cfgset = cfgsets[0]
        for d in cfgset.configuration_data:
            if d.name == name:
                return any.from_any(d.value)
        return None

    ##
    # \brief activate this component
    # \param self this object
    # \param ec execution context used to activate this component
    # \param timeout maximum duration to wait for activation
    # \return True if activated successfully, False otherwise
    def start(self, ec=None, timeout=3.0):
        if ec == None:
            ec = self.ec
        if ec != None:
            ec.activate_component(self.ref)
            tm = 0 
            while tm < timeout:
                if self.isActive(ec):
                    return True
                time.sleep(0.01)
                tm += 0.01
        return False

    ##
    # \brief deactivate this component
    # \param self this object
    # \param ec execution context used to deactivate this component
    # \param timeout maximum duration to wait for deactivation
    # \return True if deactivated successfully, False otherwise
    def stop(self, ec=None, timeout=3.0):
        if ec == None:
            ec = self.ec
        if ec != None:
            ec.deactivate_component(self.ref)
            tm = 0
            while tm < timeout:
                if self.isInactive(ec):
                    return True
                time.sleep(0.01)
                tm += 0.01
        return False

    ##
    # \brief get life cycle state of the main execution context
    # \param self this object
    # \param ec execution context from which life cycle state is obtained
    # \return one of LifeCycleState value or None if the main execution context is not set
    def getLifeCycleState(self, ec=None):
        if ec == None:
            ec = self.ec
        if ec != None:
            return ec.get_component_state(self.ref)
        else:
            return None

    ##
    # \brief check the main execution context is active or not
    # \param ec execution context
    # \retval 1 this component is active
    # \retval 0 this component is not active
    def isActive(self, ec=None):
        return RTC.ACTIVE_STATE == self.getLifeCycleState(ec)

    ##
    # \brief check the main execution context is inactive or not
    # \param ec execution context
    # \retval 1 this component is inactive
    # \retval 0 this component is not inactive
    def isInactive(self, ec=None):
        return RTC.INACTIVE_STATE == self.getLifeCycleState(ec)

    ##
    # \brief get instance name
    # \return instance name
    def name(self):
        cprof = self.ref.get_component_profile()
        return cprof.instance_name

##
# \brief wrapper class of RTCmanager
#
class RTCmanager:
    ##
    # \brief constructor
    # \param self this object
    # \param ref IOR of RTCmanager
    def __init__(self, ref):
        self.ref = ref
        uname = os.uname()[0]
        if uname == "Darwin":
            self.soext = ".dylib"
        else:
            self.soext = ".so"

    ##
    # \brief load RT component factory
    # \param self this object
    # \param basename basename of the shared library
    # \param initfunc a function called when the shared library is loaded. If
    #  not specified, basename+"Init" is called.
    def load(self, basename, initfunc=""):
        path = basename + self.soext
        if initfunc == "":
            basename + "Init"
        try:
            self.ref.load_module(path, initfunc)
        except:
            print("failed to load", path)

    ##
    # \brief create an instance of RT component
    # \param self this object
    # \param module name of RT component factory
    # \param name name of RT component instance
    # \return an object of RTcomponent
    def create(self, module, name=None):
        if name != None:
            rtc = findRTC(name)
            if rtc != None:
                print('RTC named "' + name + '" already exists.')
                return rtc
        args = module
        if name != None:
            args += '?instance_name=' + name
        ref = self.ref.create_component(args)
        if ref == None:
            return None
        else:
            return RTcomponent(ref)

    ##
    # \brief create an instance of RT component
    # \param self this object
    # \param name name of RT component instance
    def delete(self, name):
        # ref = self.ref.delete_component(name)
        ref = findRTC(name).ref.exit() # delte_component did not actually kill component, so use rtc.exit https://github.com/fkanehiro/hrpsys-base/pull/512#issuecomment-80430387
        if ref == RTC_OK:
            return True
        else:
            return False

    ##
    # \brief get list of factory names
    # \return list of factory names
    def get_factory_names(self):
        fs = []
        fps = self.ref.get_factory_profiles()
        for afp in fps:
            for p in afp.properties:
                if p.name == "implementation_id":
                    fs.append(any.from_any(p.value))
        return fs

    ##
    # \brief get list of components
    # \return list of components
    def get_components(self):
        cs = []
        crefs = self.ref.get_components()
        for cref in crefs:
            c = RTcomponent(cref)
            cs.append(c)
        return cs

    ##
    # \brief restart Manager
    def restart(self):
        self.ref.shutdown()
        time.sleep(1)

##
# \brief unbind an object reference 
# \param name name of the object
# \param kind kind of the object
#
def unbindObject(name, kind):
    nc = NameComponent(name, kind)
    path = [nc]
    rootnc.unbind(path)
    return None

##
# \brief initialize ORB 
#
def initCORBA():
    global rootnc, orb, nshost, nsport

    # from omniorb's document
    # When CORBA::ORB_init() is called, the value for each configuration
    # parameter is searched for in the following order:
    #  Command line arguments
    #  Environment variables
    # so init_CORBA will follow this order
    # first check command line argument
    try:
        n = sys.argv.index('-ORBInitRef')
        (nshost, nsport) = re.match(
            'NameService=corbaloc:iiop:(\w+):(\d+)/NameService', sys.argv[n + 1]).group(1, 2)
    except:
        if not nshost:
            nshost = socket.gethostname()
        if not nsport:
            nsport = 15005

    print("configuration ORB with %s:%s"%(nshost, nsport))
    os.environ['ORBInitRef'] = 'NameService=corbaloc:iiop:%s:%s/NameService' % \
                               (nshost, nsport)

    try:
        orb = CORBA.ORB_init(sys.argv, CORBA.ORB_ID)
        nameserver = orb.resolve_initial_references("NameService")
        rootnc = nameserver._narrow(CosNaming.NamingContext)
    except omniORB.CORBA.ORB.InvalidName:
        _, e, _ = sys.exc_info()
        sys.exit('[ERROR] Invalide Name (hostname=%s).\n' % (nshost) +
                 'Make sure the hostname is correct.\n' + str(e))
    except omniORB.CORBA.TRANSIENT:
        _, e, _ = sys.exc_info()
        sys.exit('[ERROR] Connection Failed with the Nameserver (hostname=%s port=%s).\n' % (nshost, nsport) +
                 'Make sure the hostname is correct and the Nameserver is running.\n' + str(e))
    except Exception:
        _, e, _ = sys.exc_info()
        print(str(e))

    return None

##
# \brief get root naming context 
# \param corbaloc location of NamingService
# \return root naming context
#
def getRootNamingContext(corbaloc):
    props = System.getProperties()

    args = ["-ORBInitRef", corbaloc]
    orb = ORB.init(args, props)

    nameserver = orb.resolve_initial_references("NameService")
    return NamingContextHelper.narrow(nameserver)

##
# \brief get IOR of the object
# \param name name of the object
# \param kind kind of the object
# \param rnc root naming context. If it is not specified, global variable
# rootnc is used
# \return IOR of the object
# 
def findObject(name, kind="", rnc=None):
    nc = CosNaming.NameComponent(name, kind)
    path = [nc]
    if not rnc:
        rnc = rootnc
        if not rnc:
            print("[ERROR] findObject(%r,kind=%r,rnc=%r) rootnc is not found" % (name, kind, rnc))
    return rnc.resolve(path)

##
# \brief get RTCmanager
# \param hostname hostname where rtcd is running
# \param rnc root naming context. If it is not specified, global variable rootnc
# is used
# \return an object of RTCmanager
#
def findRTCmanager(hostname=None, rnc=None):
    if not rootnc:
        print("[ERROR] findRTCmanager(hostname=%r,rnc=%r) rootnc is not defined, need to call initCORBA()" % \
              (hostname, rnc))
    if not hostname:
        hostname = nshost
        cxt = None
    if not hostname:
        hostname = socket.gethostname()
    try:
        socket.gethostbyaddr(hostname)
    except Exception:
        _, e, _ = sys.exc_info()
        sys.exit('[ERROR] %s\n' % (str(e)) + '[ERROR] Could not get hostname for %s\n' % (hostname) +
                 '[ERROR] Make sure that you set the target hostname and address in DNS or /etc/hosts in linux/unix.')


    def getManagerFromNS(hostname, mgr=None):
        try:
            obj = findObject("manager", "mgr", findObject(hostname, "host_cxt", rnc))
            mgr = RTCmanager(obj._narrow(RTM.Manager))
        except:
            mgr = None
        return mgr

    def getManagerDirectly(hostname, mgr=None):
        global orb
        corbaloc = "corbaloc:iiop:" + hostname + ":2810/manager"
        try:
            obj = orb.string_to_object(corbaloc)
            mgr = RTCmanager(obj._narrow(RTM.Manager))
        except:
            mgr = None
        return mgr

    try:
        import CORBA
    except:
        print('import CORBA failed in findRTCmanager and neglect it for old python environment.')
    # fqdn
    mgr = None
    hostnames = [hostname, hostname.split(".")[0],
                 socket.gethostbyaddr(hostname)[0],
                 socket.gethostbyaddr(hostname)[0].split(".")[0]]
    for h in hostnames:
        mgr = getManagerDirectly(h) or getManagerFromNS(h)
        if mgr:
            return mgr
    print("Manager not found")
    return None


##
# \brief get RT component
# \param name name of the RT component
# \param rnc root naming context. If it is not specified, global variable
# rootnc is used
# \return an object of RTcomponent
#
def findRTC(name, rnc=None):
    try:
        obj = findObject(name, "rtc", rnc)
        try:
            rtc = RTcomponent(obj._narrow(RTC.RTObject))
        except TypeError:
            rtc = RTcomponent(obj._narrow(RTC.DataFlowComponent))
        cxts = rtc.ref.get_participating_contexts()
        if len(cxts) > 0:
            rtc.ec = cxts[0]
        return rtc
    except:
        return None

##
# \brief get a port of RT component
# \param rtc an object of RTcomponent
# \param name name of the port
# \return IOR of the port if the port is found, None otherwise
#
def findPort(rtc, name):
    ports = rtc.get_ports()
    cprof = rtc.get_component_profile()
    portname = cprof.instance_name + "." + name
    for p in ports:
        prof = p.get_port_profile()
        if prof.name == portname:
            return p
    return None

##
# \brief set up execution context of the first RTC so that RTCs are executed
# sequentially
# \param rtcs sequence of RTCs
# \param stopEC whether stop owned ECs of slave components
#
def serializeComponents(rtcs, stopEC=True):
    if len(rtcs) < 2:
        return
    ec = rtcs[0].ec
    for rtc in rtcs[1:]:
        try:
            if not ec._is_equivalent(rtc.ec):
                if stopEC:
                    rtc.ec.stop()
                if ec.add_component(rtc.ref) == RTC.RTC_OK:
                    rtc.ec = ec
                else:
                    print('error in add_component()')
            else:
                print(rtc.name() + 'is already serialized')
        except Exception:
            _, e, _ = sys.exc_info()
            print("[rtm.py] \033[31m   error in serialize %s of %s %s\033[0m" % (rtc.name(),  [[r, r.name()] for r in rtcs], str(e)))
            raise e

##
# \brief check two ports are connected or not
# \retval True connected
# \retval False not connected
#
def isConnected(outP, inP):
    op = outP.get_port_profile()
    for con_prof in op.connector_profiles:
        ports = con_prof.ports
        if len(ports) == 2 and outP._is_equivalent(ports[0]) and \
           inP._is_equivalent(ports[1]):
            return True
    return False

##
# \brief disconnect ports
# \param outP IOR of outPort
# \param inP IOR of inPort
# \return True disconnected successfully, False otherwise
#
def disconnectPorts(outP, inP):
    op = outP.get_port_profile()
    iname = inP.get_port_profile().name
    for con_prof in op.connector_profiles:
        ports = con_prof.ports
        if len(ports) == 2:
            pname = ports[1].get_port_profile().name
            if pname == iname:
                print('[rtm.py]    Disconnect %s - %s' %(op.name, iname))
                outP.disconnect(con_prof.connector_id)
                return True
    return False

##
# \brief get data type of a port
# \param port IOR of port
# \return data type
#
def dataTypeOfPort(port):
    prof = port.get_port_profile()
    prop = prof.properties
    for p in prop:
        if p.name == "dataport.data_type":
            return any.from_any(p.value)
    return None

##
# \brief connect ports
# \param outP IOR of outPort 
# \param inPs an IOR or a list of IORs of inPort
# \param subscription subscription type. "flush", "new" or "periodic"
# \param dataflow dataflow type. "Push" or "Pull"
# \param bufferlength length of data buffer
# \param rate communication rate for periodic mode[Hz]
#
def connectPorts(outP, inPs, subscription="flush", dataflow="Push", bufferlength=1, rate=1000, pushpolicy="new", interfaceType="corba_cdr"):
    if not isinstance(inPs, list):
        inPs = [inPs]
    if not outP:
        print('[rtm.py] \033[31m   Failed to connect %s to %s(%s)\033[0m' % \
              (outP, [inP.get_port_profile().name if inP else inP for inP in inPs], inPs))
        return
    for inP in inPs:
        if not inP:
            print('[rtm.py] \033[31m   Failed to connect %s to %s(%s)\033[0m' % \
                  (outP.get_port_profile().name, inP, inPs))
            continue
        if isConnected(outP, inP) == True:
            print('[rtm.py]      %s and %s are already connected' % \
                  (outP.get_port_profile().name, inP.get_port_profile().name))
            continue
        if dataTypeOfPort(outP) != dataTypeOfPort(inP):
            print('[rtm.py] \033[31m     %s and %s have different data types\033[0m' % \
                  (outP.get_port_profile().name, inP.get_port_profile().name))
            continue
        nv1 = SDOPackage.NameValue("dataport.interface_type", any.to_any(interfaceType))
        nv2 = SDOPackage.NameValue("dataport.dataflow_type", any.to_any(dataflow))
        nv3 = SDOPackage.NameValue("dataport.subscription_type", any.to_any(subscription))
        nv4 = SDOPackage.NameValue("dataport.buffer.length", any.to_any(str(bufferlength)))
        nv5 = SDOPackage.NameValue("dataport.publisher.push_rate", any.to_any(str(rate)))
        nv6 = SDOPackage.NameValue("dataport.publisher.push_policy", any.to_any(pushpolicy))
        nv7 = SDOPackage.NameValue("dataport.data_type", any.to_any(dataTypeOfPort(outP)))
        con_prof = RTC.ConnectorProfile("connector0", "", [outP, inP],
                                        [nv1, nv2, nv3, nv4, nv5, nv6, nv7])
        print('[rtm.py]    Connect ' + outP.get_port_profile().name + ' - ' + \
              inP.get_port_profile().name+' (dataflow_type='+dataflow+', subscription_type='+ subscription+', bufferlength='+str(bufferlength)+', push_rate='+str(rate)+', push_policy='+pushpolicy+')')
        ret, prof = inP.connect(con_prof)
        if ret != RTC.RTC_OK:
            print("failed to connect")
            continue
        # confirm connection
        if isConnected(outP, inP) == False:
            print("connet() returned RTC_OK, but not connected")

##
# \brief convert data into CDR format
# \param data data to be converted
# \return converted data in CDR format
#
def data2cdr(data):
    return cdrMarshal(any.to_any(data).typecode(), data, True)

##
# \brief get class object from class name
# \param fullname class name
# \return class object
#
def classFromString(fullname):
    component_path = fullname.split('.')
    package_path = component_path[:-1]
    package_name = ".".join(package_path)
    class_name = component_path[-1]
    __import__(str(package_name))
    return getattr(sys.modules[package_name], class_name)

##
# \brief convert data from CDR format
# \param cdr in CDR format 
# \param classname class name of the data
# \return converted data
#
def cdr2data(cdr, classname):
    return cdrUnmarshal(any.to_any(classFromString(classname)).typecode(), cdr, True)

##
# \brief write data to a data port	
# \param port reference of data port
# \param data data to be written
# \param tm If disconnect==True, a connection to write data is disconnected
# after this time
# \param disconnect If True, a connection is disconnected after tm and if not,
# the connection must be disconnected by a user
#
def writeDataPort(port, data, tm=1.0, disconnect=True):
    nv1 = SDOPackage.NameValue("dataport.interface_type", any.to_any("corba_cdr"))
    nv2 = SDOPackage.NameValue("dataport.dataflow_type", any.to_any("Push"))
    nv3 = SDOPackage.NameValue("dataport.subscription_type", any.to_any("flush"))
    con_prof = RTC.ConnectorProfile("connector0", "", [port], [nv1, nv2, nv3])
    ret, prof = port.connect(con_prof)
    if ret != RTC.RTC_OK:
        print("failed to connect")
        return None
    for p in prof.properties:
        if p.name == 'dataport.corba_cdr.inport_ior':
            ior = any.from_any(p.value)
            obj = orb.string_to_object(ior)
            inport = obj._narrow(InPortCdr)
            cdr = data2cdr(data)
            if inport.put(cdr) != OpenRTM.PORT_OK:
                print("failed to put")
            if disconnect:
                time.sleep(tm)
                port.disconnect(prof.connector_id)
            else:
                return prof.connector_id
    return None

##
# \brief read data from a data port	
# \param port reference of data port
# \param timeout timeout[s] 
# \return data
#
def readDataPort(port, timeout=1.0):
    pprof = port.get_port_profile()
    for prop in pprof.properties:
        if prop.name == "dataport.data_type":
            classname = any.from_any(prop.value)
        if prop.name == "dataport.data_value":
            return any._to_tc_value(prop.value)[1]
    nv1 = SDOPackage.NameValue("dataport.interface_type", any.to_any("corba_cdr"))
    nv2 = SDOPackage.NameValue("dataport.dataflow_type", any.to_any("Pull"))
    nv3 = SDOPackage.NameValue("dataport.subscription_type", any.to_any("flush"))
    con_prof = RTC.ConnectorProfile("connector0", "", [port], [nv1, nv2, nv3])
    ret, prof = port.connect(con_prof)
    if ret != RTC.RTC_OK:
        print("failed to connect")
        return None
    for p in prof.properties:
        # print(p.name)
        if p.name == 'dataport.corba_cdr.outport_ior':
            ior = any.from_any(p.value)
            obj = orb.string_to_object(ior)
            outport = obj._narrow(OutPortCdr)
            tm = 0
            while tm < timeout:
                try:
                    ret, data = outport.get()
                    if ret == OpenRTM.PORT_OK:
                        port.disconnect(prof.connector_id)
                        tokens = classname.split(':')
                        if len(tokens) == 3:  # for 1.1?
                            classname = tokens[1].replace('/', '.')
                        return cdr2data(data, classname)
                except:
                    pass
                time.sleep(0.1)
                tm = tm + 0.1

    port.disconnect(prof.connector_id)
    return None


##
# \brief get a service of RT component
# \param rtc IOR of RT component
# \param port_name port name of the port which provides the service
# \param type_name type name of the service
# \param instance_name name of the service
# \return IOR of the service
#
def findService(rtc, port_name, type_name, instance_name):
    if port_name == "":
        prof = rtc.ref.get_component_profile()
        # print("RTC name:",prof.instance_name)
        port_prof = prof.port_profiles
    else:
        p = rtc.port(port_name)
        if p == None:
            print("can't find a port named" + port_name)
            return None
        else:
            port_prof = [p.get_port_profile()]
    port = None
    for pp in port_prof:
        # print("name:", pp.name)
        ifs = pp.interfaces
        for aif in ifs:
            #print("IF name:", aif.instance_name)
            #print("IF type:", aif.type_name)
            if aif.instance_name == instance_name and \
               (type_name == "" or aif.type_name == type_name) and \
               aif.polarity == PROVIDED:
                port = pp.port_ref
    if port == None:
        print("can't find a service named", instance_name)
        return None
    con_prof = RTC.ConnectorProfile("noname", "", [port], [])
    ret, con_prof = port.connect(con_prof)
    ior = any.from_any(con_prof.properties[0].value)
    return orb.string_to_object(ior)

##
# \brief update default configuration set
# \param rtc IOR of RT component
# \param nvlist list of pairs of name and value
# \return True if all values are set correctly, False otherwise
#
def setConfiguration(rtc, nvlist):
    ret = True
    cfg = rtc.get_configuration()
    cfgsets = cfg.get_configuration_sets()
    if len(cfgsets) == 0:
        print("configuration set is not found")
        return
    cfgset = cfgsets[0]
    for nv in nvlist:
        name = nv[0]
        value = nv[1]
        found = False
        for d in cfgset.configuration_data:
            if d.name == name:
                d.value = any.to_any(value)
                cfg.set_configuration_set_values(cfgset)
                found = True
                break
        if not found:
            ret = False
    cfg.activate_configuration_set('default')
    return ret

##
# \brief narrow ior
# \param ior ior
# \param klass class name 
# \param package package where the class is defined
#
def narrow(ior, klass, package="OpenHRP"):
    return ior._narrow(getattr(sys.modules[package], klass))

##
# \brief check if jython or python
# \return True if jython
#
def isJython():
    return sys.version.count("GCC") == 0


if __name__ == '__main__':
    initCORBA()

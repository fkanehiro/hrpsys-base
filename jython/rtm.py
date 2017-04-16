##
# \file rtm.py
# \brief jython library to control RT components
#
from org.omg.CORBA import *
from org.omg.CosNaming import *
from org.omg.CosNaming.NamingContextPackage import *
from com.sun.corba.se.impl.encoding import EncapsOutputStream

from java.lang import System, Class

from RTC import *
from RTM import *
from OpenRTM import *
from _SDOPackage import *

import string, math, socket, time, sys

##
# \brief root naming context
#
rootnc = None

##
# \brief hostname where naming service is running
#
nshost = None
 
##
# \brief wrapper class of RT component
#
class RTcomponent:
	##
	# \brief constructor
	# \param self this object
	# \param ref IOR of RT component
	#
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
	def setConfiguration(self, nvlist):
		setConfiguration(self.ref, nvlist)

        ##
	# \brief update value of the default configuration set
	# \param self this object	
	# \param name name of the property
	# \param value new value of the property
	def setProperty(self, name, value):
		self.setConfiguration([[name, value]])

        ##
	# \brief get value of the property in the default configuration set
	# \param self this object	
	# \param name name of the property
        # \return value of the property or None if the property is not found
	def getProperty(self, name):
		cfg = self.ref.get_configuration()
		cfgsets = cfg.get_configuration_sets()
		if len(cfgsets) == 0:
			print "configuration set is not found"
			return None
		cfgset = cfgsets[0]
		for d in cfgset.configuration_data:
			if d.name == name:
				if p.value.type().kind().value() == TCKind._tk_string:
					return p.value.extract_string()
				elif p.value.type().kind().value() == TCKind._tk_wstring:
					return p.value.extract_wstring()
		return None		

	##
	# \brief show list of property names and values
	# \param self this object
	def properties(self):
		cfg = self.ref.get_configuration()
		cfgsets = cfg.get_configuration_sets()
		if len(cfgsets) == 0:
			print "configuration set is not found"
			return
		cfgset = cfgsets[0]
		for d in cfgset.configuration_data:
			if d.value.type().kind().value() == TCKind._tk_string:
				print d.name,":",d.value.extract_string()
			elif d.value.type().kind().value() == TCKind._tk_wstring:
				print d.name,":",d.value.extract_wstring()
		

	##
	# \brief activate this component
	# \param self this object
	# \param ec execution context used to activate this component
	def start(self, ec=None):
		if ec == None:
			ec = self.ec
		if ec != None:
			ec.activate_component(self.ref)
			while self.isInactive(ec):
				time.sleep(0.01)

	##
	# \brief deactivate this component
	# \param self this object
	# \param ec execution context used to deactivate this component
	def stop(self, ec=None):
		if ec == None:
			ec = self.ec
		if ec != None:
			ec.deactivate_component(self.ref)
			while self.isActive(ec):
				time.sleep(0.01)

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
		return LifeCycleState.ACTIVE_STATE == self.getLifeCycleState(ec)

	##
	# \brief check the main execution context is inactive or not
	# \param ec execution context
        # \retval 1 this component is inactive
        # \retval 0 this component is not inactive
        def isInactive(self, ec=None):
		return LifeCycleState.INACTIVE_STATE == self.getLifeCycleState(ec)

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
	#
	def __init__(self, ref):
		self.ref = ref
		osname = System.getProperty("os.name")
		# assuming jython and rtcd are running on the same OS
		if osname == "Mac OS X":
			self.soext = ".dylib"
		else: 
			self.soext = ".so"
	
	##
	# \brief load RT component factory
	# \param self this object
	# \param basename common part of path of the shared library and the initialize function. path is generated by basename+".so" and the initialize function is generated by basename+"Init".
	#
	def load(self, basename):
		path = basename+self.soext
		initfunc = basename+"Init"
		try:
			self.ref.load_module(path, initfunc)
		except:
			print "failed to load",path

	##
	# \brief create an instance of RT component
	# \param self this object
	# \param module name of RT component factory
        # \param name name of RT component instance
	# \return an object of RTcomponent
	def create(self, module,name=None):
		if name != None:
			rtc = findRTC(name)
			if rtc != None:
				print 'RTC named "',name,'" already exists.'
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
	# \brief get list of factory names
        # \return list of factory names
	def get_factory_names(self):
		fs = []
		fps = self.ref.get_factory_profiles()
		for afp in fps:
			for p in afp.properties:
				if p.name == "implementation_id":
					if p.value.type().kind().value() == TCKind._tk_string:
						fs.append(p.value.extract_string())
					elif p.value.type().kind().value() == TCKind._tk_wstring:
						fs.append(p.value.extract_wstring())

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
	global rootnc, nshost, orb
	props = System.getProperties()
	
	args = string.split(System.getProperty("NS_OPT"))
	nshost = System.getProperty("NS_OPT").split(':')[2]
	if nshost == "localhost" or nshost == "127.0.0.1":
		nshost = socket.gethostname()
	print 'nshost =',nshost
	orb = ORB.init(args, props)

	nameserver = orb.resolve_initial_references("NameService");
	rootnc = NamingContextHelper.narrow(nameserver);
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

	nameserver = orb.resolve_initial_references("NameService");
	return NamingContextHelper.narrow(nameserver);

##
# \brief get IOR of the object
# \param name name of the object
# \param kind kind of the object
# \param rnc root naming context. If it is not specified, global variable rootnc is used
# \return IOR of the object
# 
def findObject(name, kind="", rnc=None):
	nc = NameComponent(name,kind)
	path = [nc]
	if not rnc:
		rnc = rootnc
	return rnc.resolve(path)

##
# \brief get RTCmanager
# \param hostname hostname where rtcd is running
# \param rnc root naming context. If it is not specified, global variable rootnc is used
# \return an object of RTCmanager
#
def findRTCmanager(hostname=None, rnc=None):
	if not hostname:
		hostname = nshost
	try:
		try:
			cxt = findObject(hostname, "host_cxt", rnc)
		except:
			hostname = socket.gethostbyaddr(hostname)[0]
			cxt = findObject(hostname, "host_cxt", rnc)
		obj = findObject("manager","mgr",cxt)
		return RTCmanager(ManagerHelper.narrow(obj))
	except:
		print "exception in findRTCmanager("+hostname+")"

##
# \brief get RT component
# \param name name of the RT component
# \param rnc root naming context. If it is not specified, global variable rootnc is used
# \return an object of RTcomponent
#
def findRTC(name, rnc=None):
	try:
		obj = findObject(name, "rtc", rnc)
		rtc = RTcomponent(RTObjectHelper.narrow(obj))
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
	portname = cprof.instance_name+"."+name
	for p in ports:
		prof = p.get_port_profile()
		if prof.name == portname:
			return p
	return None 

##
# \brief set up execution context of the first RTC so that RTCs are executed sequentially
# \param rtcs sequence of RTCs
# \param stopEC whether stop owned ECs of slave components
#
def serializeComponents(rtcs, stopEC=True):
	if len(rtcs) < 2:
		return
	ec = rtcs[0].ec
	for rtc in rtcs[1:]:
		if ec != rtc.ec: 
			if stopEC:
				rtc.ec.stop()
			if ec.add_component(rtc.ref) == ReturnCode_t.RTC_OK:
				rtc.ec = ec
			else:
				print 'error in add_component()'
		else:
			print 'already serialized'

##
# \brief check two ports are connected or not
# \retval True connected
# \retval False not connected
def isConnected(outP, inP):
	op = outP.get_port_profile()
	for con_prof in op.connector_profiles:
		ports = con_prof.ports
		if len(ports) == 2 and outP == ports[0] and inP == ports[1]:
			return True
	return False

##
# \brief disconnect ports
# \param outP IOR of outPort
# \param inP IOR of inPort
def disconnectPorts(outP, inP):
	op = outP.get_port_profile()
	for con_prof in op.connector_profiles:
		ports = con_prof.ports
		if len(ports) == 2 and outP == ports[0] and inP == ports[1]:
			outP.disconnect(con_prof.connector_id)
	return

##
# \brief get data type of a port
# \param port IOR of port
# \return data type
def dataTypeOfPort(port):
	prof = port.get_port_profile()
	prop = prof.properties
	for p in prop:
		if p.name == "dataport.data_type":
			if p.value.type().kind().value() == TCKind._tk_string:
				return p.value.extract_string()
			elif p.value.type().kind().value() == TCKind._tk_wstring:
				return p.value.extract_wstring()

	return None

##
# \brief connect ports
# \param outP IOR of outPort 
# \param inPs an IOR or a list of IORs of inPort
# \param subscription subscription type. "flush", "new" or "periodic"
# \param dataflow dataflow type. "Push" or "Pull"
# \param bufferlength length of data buffer
# \param rate rate[Hz] for subscription type "periodic"
#
def connectPorts(outP, inPs, subscription="flush", dataflow="Push", bufferlength=1, rate=1000):
	if not isinstance(inPs, list):
		inPs = [inPs]
	for inP in inPs: 
		if isConnected(outP, inP) == True:
			print outP.get_port_profile().name,'and',inP.get_port_profile().name,'are already connected'
			continue
		if dataTypeOfPort(outP) != dataTypeOfPort(inP):
			print outP.get_port_profile().name,'and',inP.get_port_profile().name,'have different data types'
			continue
			
		con_prof = ConnectorProfile()
		con_prof.connector_id = ""
		con_prof.name = "connector0"
		con_prof.ports = [outP, inP]
		#
		nv1 = NameValue()
		nv1.name = "dataport.interface_type";
		a1 = orb.create_any()
		a1.insert_string("corba_cdr")
		nv1.value = a1
		#
		nv2 = NameValue()
		nv2.name = "dataport.dataflow_type"
		a2 = orb.create_any()
		a2.insert_string(dataflow)
		nv2.value = a2
		#
		nv3 = NameValue()
		nv3.name = "dataport.subscription_type"
		a3 = orb.create_any()
		a3.insert_string(subscription)
		nv3.value = a3
		#
		nv4 = NameValue()
		nv4.name = "dataport.buffer.length"
		a4 = orb.create_any()
		a4.insert_string(str(bufferlength))
		nv4.value = a4
		#
		nv5 = NameValue()
		nv5.name = "dataport.publisher.push_rate"
		a5 = orb.create_any()
		a5.insert_string(str(rate))
		nv5.value = a5
		#
		con_prof.properties = [nv1, nv2, nv3, nv4, nv5]
		con_prof_holder = ConnectorProfileHolder()
		con_prof_holder.value = con_prof
		if inP.connect(con_prof_holder) != ReturnCode_t.RTC_OK:
			print "failed to connect(",outP.get_port_profile().name,'<->',inP.get_port_profile().name,")"
			continue
		# confirm connection
		if isConnected(outP, inP) == False:
			print "connet() returned RTC_OK, but not connected"

##
# \brief convert data into CDR format
# \param data data to be converted
# \return converted data in CDR format
#
def data2cdr(data):
	holder = Class.forName(data.getClass().getCanonicalName()+"Holder",
			       True, data.getClass().getClassLoader())
	streamable = holder.newInstance()
	#field = holder.getField("value")
	#field.set(streamable, data)
	streamable.value = data;
	strm = EncapsOutputStream(orb, True)
	streamable._write(strm)
	return strm.toByteArray()

##
# \brief convert data from CDR format
# \param cdr in CDR format 
# \param classname class name of the data
# \return converted data
#
def cdr2data(cdr, classname):
	ostrm = EncapsOutputStream(orb, True)
	ostrm.write_octet_array(cdr, 0, len(cdr))
	istrm = ostrm.create_input_stream()
	holder = Class.forName("RTC."+classname+"Holder", True, Manager.getClassLoader())
	streamable = holder.newInstance()
	streamable._read(istrm)
	return streamable.value

##
# \brief write data to a data port	
# \param port reference of data port
# \param data data to be written
# \param tm after this time, a connection to write data is disconnected
#
def writeDataPort(port, data, tm=1.0):
	con_prof = ConnectorProfile()
	con_prof.connector_id = ""
	con_prof.name = "connector0"
	con_prof.ports = [port]
	#
	nv1 = NameValue()
	nv1.name = "dataport.interface_type";
	a1 = orb.create_any()
	a1.insert_string("corba_cdr")
	nv1.value = a1
	#
	nv2 = NameValue()
	nv2.name = "dataport.dataflow_type"
	a2 = orb.create_any()
	a2.insert_string("Push")
	nv2.value = a2
	#
	nv3 = NameValue()
	nv3.name = "dataport.subscription_type"
	a3 = orb.create_any()
	a3.insert_string("flush")
	nv3.value = a3
	#
	con_prof.properties = [nv1, nv2, nv3]
	con_prof_holder = ConnectorProfileHolder()
	con_prof_holder.value = con_prof
	if port.connect(con_prof_holder) != ReturnCode_t.RTC_OK:
		print "failed to connect"
		return None
	for p in con_prof_holder.value.properties:
		if p.name == 'dataport.corba_cdr.inport_ior':
			if p.value.type().kind().value() == TCKind._tk_string:
				ior = p.value.extract_string()
			elif p.value.type().kind().value() == TCKind._tk_wstring:
				ior = p.value.extract_wstring()
			obj = orb.string_to_object(ior)
			inport = InPortCdrHelper.narrow(obj)
			cdr = data2cdr(data)
			if inport.put(cdr) != PortStatus.PORT_OK:
				print "failed to put"
			time.sleep(tm)
			port.disconnect(con_prof_holder.value.connector_id)
	return None		
			
		
##
# \brief read data from a data port	
# \param port reference of data port
# \param timeout timeout[s] 
# \return data 
def readDataPort(port, timeout = 1.0):
	pprof = port.get_port_profile()
	for prop in pprof.properties:
		if prop.name == "dataport.data_type":
			if prop.value.type().kind().value() == TCKind._tk_string:
				classname = prop.value.extract_string()
			elif prop.value.type().kind().value() == TCKind._tk_wstring:
				classname = prop.value.extract_wstring()
			break;
	con_prof = ConnectorProfile()
	con_prof.connector_id = ""
	con_prof.name = "connector0"
	con_prof.ports = [port]
	#
	nv1 = NameValue()
	nv1.name = "dataport.interface_type";
	a1 = orb.create_any()
	a1.insert_string("corba_cdr")
	nv1.value = a1
	#
	nv2 = NameValue()
	nv2.name = "dataport.dataflow_type"
	a2 = orb.create_any()
	a2.insert_string("Pull")
	nv2.value = a2
	#
	nv3 = NameValue()
	nv3.name = "dataport.subscription_type"
	a3 = orb.create_any()
	a3.insert_string("flush")
	nv3.value = a3
	#
	con_prof.properties = [nv1, nv2, nv3]
	con_prof_holder = ConnectorProfileHolder()
	con_prof_holder.value = con_prof
	if port.connect(con_prof_holder) != ReturnCode_t.RTC_OK:
		print "failed to connect"
		return None
	for p in con_prof_holder.value.properties:
		#print p.name
		if p.name == 'dataport.corba_cdr.outport_ior':
			if p.value.type().kind().value() == TCKind._tk_string:
				ior = p.value.extract_string()
			elif p.value.type().kind().value() == TCKind._tk_wstring:
				ior = p.value.extract_wstring()
			obj = orb.string_to_object(ior)
			outport = OutPortCdrHelper.narrow(obj)
			cdr = CdrDataHolder()
			tm = 0
			while tm < timeout:
				try:
					outport.get(cdr)
					if len(cdr.value) > 0:
						port.disconnect(con_prof_holder.value.connector_id)
						return cdr2data(cdr.value, classname)
				except:
					pass
				time.sleep(0.1)
				tm = tm + 0.1

	port.disconnect(con_prof_holder.value.connector_id)
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
	        #print "RTC name:",prof.instance_name
		port_prof = prof.port_profiles
	else:
		p = rtc.port(port_name)
		if p == None:
			print "can't find a port named",port_name
			return None
		else:
			port_prof = [p.get_port_profile()]
	port = None
	for pp in port_prof:
		#print "name:",pp.name
		ifs = pp.interfaces
		for aif in ifs:
			#print "IF name:",aif.instance_name
			#print "IF type:",aif.type_name
			if aif.instance_name == instance_name and (type_name == "" or aif.type_name == type_name):
				port = pp.port_ref
	if port == None:
		print "can't find a service named",instance_name
		return None
        #print port
	con_prof = ConnectorProfile()	
	con_prof.name = "noname"
	con_prof.connector_id = ""
	con_prof.ports = [port]
	con_prof.properties = []
	con_prof_holder = ConnectorProfileHolder()
	con_prof_holder.value = con_prof
	port.connect(con_prof_holder)
	if con_prof_holder.value.properties[0].value.type().kind().value() == TCKind._tk_string:
		ior = con_prof_holder.value.properties[0].value.extract_string()
	elif con_prof_holder.value.properties[0].value.type().kind().value() == TCKind._tk_wstring:
		ior = con_prof_holder.value.properties[0].value.extract_wstring()
	port.disconnect(con_prof_holder.value.connector_id)
	return orb.string_to_object(ior)

##
# \brief update default configuration set
# \param rtc IOR of RT component
# \param nvlist list of pairs of name and value
#
def setConfiguration(rtc, nvlist):
	cfg = rtc.get_configuration()
	cfgsets = cfg.get_configuration_sets()
	if len(cfgsets) == 0:
		print "configuration set is not found"
		return
	cfgset = cfgsets[0]
	for nv in nvlist:
		name = nv[0]
		value = nv[1]
		found = False
		for d in cfgset.configuration_data:
			if d.name == name:
				d.value.insert_string(value)
				cfg.set_configuration_set_values(cfgset)
				found = True
				break;
		if not found:
			print "no such property(",name,")"
	cfg.activate_configuration_set('default')

##
# \brief narrow ior
# \param ior ior
# \param klass class name 
# \param package package where the class is defined
def narrow(ior, klass, package="OpenHRP"):
	return getattr(sys.modules[package], klass+"Helper").narrow(ior)

##
# \brief check if jython or python
# \return True if jython
def isJython():
	return sys.version.count("GCC") == 0

initCORBA()

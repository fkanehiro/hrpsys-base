#include <rtm/CorbaNaming.h>
#include "OpenRTMUtil.h"

int connectPorts(RTC::PortService_ptr outPort, RTC::PortService_ptr inPort)
{
    RTC::ConnectorProfileList_var connectorProfiles = inPort->get_connector_profiles();
    for(CORBA::ULong i=0; i < connectorProfiles->length(); ++i){
        RTC::ConnectorProfile& connectorProfile = connectorProfiles[i];
        RTC::PortServiceList& connectedPorts = connectorProfile.ports;
        
        for(CORBA::ULong j=0; j < connectedPorts.length(); ++j){
            RTC::PortService_ptr connectedPortRef = connectedPorts[j];
            if(connectedPortRef->_is_equivalent(outPort)){
                return 1;
            }
        }
    }
    // connect ports
    RTC::ConnectorProfile cprof;
    cprof.connector_id = "";
    cprof.name = CORBA::string_dup("connector0");
    cprof.ports.length(2);
    cprof.ports[0] = RTC::PortService::_duplicate(inPort);
    cprof.ports[1] = RTC::PortService::_duplicate(outPort);

    CORBA_SeqUtil::push_back(cprof.properties,
		       NVUtil::newNV("dataport.dataflow_type",
				     "Push"));
    CORBA_SeqUtil::push_back(cprof.properties,
		       NVUtil::newNV("dataport.interface_type",
				     "corba_cdr"));
    CORBA_SeqUtil::push_back(cprof.properties,
		       NVUtil::newNV("dataport.subscription_type",
				     "flush"));
    RTC::ReturnCode_t result = inPort->connect(cprof);

    if(result == RTC::RTC_OK)
        return 0;
    else
        return -1;
}

void activateRtc(RTC::RtcBase* pRtc)
{
    RTC::ExecutionContextList_var eclist = pRtc->get_owned_contexts();
    for(CORBA::ULong i=0; i < eclist->length(); ++i){
        if(!CORBA::is_nil(eclist[i])){
            eclist[i]->activate_component(pRtc->getObjRef());
            break;
        }
    }
}

void deactivateRtc(RTC::RtcBase* pRtc)
{
    RTC::ExecutionContextList_var eclist = pRtc->get_owned_contexts();
    for(CORBA::ULong i=0; i < eclist->length(); ++i){
        if(!CORBA::is_nil(eclist[i])){
            eclist[i]->deactivate_component(pRtc->getObjRef());
            break;
        }
    }
}

const char *getServiceIOR(RTC::RTObject_var rtc, 
                          const char *sname)
{
    const char *ior = NULL;

    RTC::PortServiceList ports;
    ports = *(rtc->get_ports());

    RTC::ComponentProfile* cprof;
    cprof = rtc->get_component_profile();
    std::string portname = std::string(cprof->instance_name) + "." + sname;

    for(unsigned int i=0; i < ports.length(); i++)
        {
            RTC::PortService_var port = ports[i];
            RTC::PortProfile* prof = port->get_port_profile();
            if(std::string(prof->name) == portname)
                {
                    RTC::ConnectorProfile connProfile;
                    connProfile.name = "noname";
                    connProfile.connector_id = "";
                    connProfile.ports.length(1);
                    connProfile.ports[0] = port;
                    port->connect(connProfile);

                    connProfile.properties[0].value >>= ior;

                    port->disconnect(connProfile.connector_id);

                    return ior;
                }
        }

    return ior;
}

void setConfiguration(RTC::RTObject_var rtc, 
                      const std::string& name, const std::string& value)
{
    SDOPackage::Configuration_ptr cfg = rtc->get_configuration();
    SDOPackage::ConfigurationSetList_var cfgsets 
        = cfg->get_configuration_sets();
    if (cfgsets->length()==0){
        std::cerr << "configuration set is not found" << std::endl;
        return;
    }
    SDOPackage::ConfigurationSet& cfgset = cfgsets[0];
    SDOPackage::NVList& nv = cfgset.configuration_data;
    for (size_t i=0; i<nv.length(); i++){
        if (std::string(nv[i].name) == name){
            nv[i].value <<= value.c_str();
            cfg->set_configuration_set_values(cfgset);
            cfg->activate_configuration_set("default");
            return;
        }
    }
    std::cerr << "can't find property(" << name << ")" << std::endl;
}

RTC::RTObject_var findRTC(const std::string &rtcName)
{
    RTC::Manager& manager = RTC::Manager::instance();
    std::string nameServer = manager.getConfig()["corba.nameservers"];
    int comPos = nameServer.find(",");
    if (comPos < 0){
        comPos = nameServer.length();
    }
    nameServer = nameServer.substr(0, comPos);
    RTC::CorbaNaming naming(manager.getORB(), nameServer.c_str());
    CosNaming::Name name;
    name.length(1);
    name[0].id = CORBA::string_dup(rtcName.c_str());
    name[0].kind = CORBA::string_dup("rtc");
    try{
        CORBA::Object_ptr obj = naming.resolve(name);
        return RTC::RTObject::_narrow(obj);
    }catch(...){
        return NULL;
    }
}


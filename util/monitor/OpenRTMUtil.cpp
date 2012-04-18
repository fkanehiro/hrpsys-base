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
                    connProfile.properties = NULL;
                    port->connect(connProfile);

                    connProfile.properties[0].value >>= ior;

                    port->disconnect(connProfile.connector_id);

                    return ior;
                }
        }

    return ior;
}

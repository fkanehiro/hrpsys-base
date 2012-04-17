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

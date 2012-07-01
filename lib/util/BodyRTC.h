#ifndef BODY_EXT_H_INCLUDED
#define BODY_EXT_H_INCLUDED

#include <hrpModel/Body.h>
#include <hrpCorba/OpenHRPCommon.hh>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include "Img.hh"

class PortHandler;

class BodyRTC : virtual public hrp::Body, public RTC::DataFlowComponentBase
{
public:
    BodyRTC(RTC::Manager* manager = &RTC::Manager::instance());
    virtual ~BodyRTC(void);

    RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id){
        std::cout << "BodyRTC::onActivated(" << ec_id << ")" << std::endl;
        return RTC::RTC_OK;
    }
    RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id){
        std::cout << "BodyRTC::onDeactivated(" << ec_id << ")" << std::endl;
        return RTC::RTC_OK;
    }

    void createInPort(const std::string &config);
    void createOutPort(const std::string &config);
    void writeDataPorts();
    void readDataPorts();
    static void moduleInit(RTC::Manager*);

private:
    static const char* bodyrtc_spec[];

    // DataInPort
    std::vector<PortHandler *> m_inports;

    // DataOutPort
    std::vector<PortHandler *> m_outports;
    int dummy;
};

typedef boost::intrusive_ptr<BodyRTC> BodyRTCPtr;

#endif

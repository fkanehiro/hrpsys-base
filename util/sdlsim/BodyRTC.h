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

class BodyRTC : public hrp::Body, public RTC::DataFlowComponentBase
{
public:
    BodyRTC(RTC::Manager* manager = &RTC::Manager::instance());
    BodyRTC(const char* url, RTC::Manager* manager = &RTC::Manager::instance());
    BodyRTC(const Body&, RTC::Manager* manager = &RTC::Manager::instance());
    BodyRTC(const BodyRTC&);
    virtual ~BodyRTC(void);

    RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id){
        std::cout << "BodyRTC::onActivated(" << ec_id << ")" << std::endl;
        return RTC::RTC_OK;
    }
    RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id){
        std::cout << "BodyRTC::onDeactivated(" << ec_id << ")" << std::endl;
        return RTC::RTC_OK;
    }

    void createDataPorts();
    void writeDataPorts();
    void readDataPorts();
    static void moduleInit(RTC::Manager*);

private:
    static const char* bodyext_spec[];

    // DataInPort
    RTC::TimedDoubleSeq m_tau;
    RTC::InPort<RTC::TimedDoubleSeq> m_tauIn;

    // DataOutPort
    RTC::TimedDoubleSeq m_q;
    std::vector<RTC::TimedAcceleration3D> m_acc;
    std::vector<RTC::TimedAngularVelocity3D> m_rate;
    std::vector<RTC::TimedDoubleSeq> m_force;

    RTC::OutPort<RTC::TimedDoubleSeq> m_qOut;
    std::vector<RTC::OutPort<RTC::TimedAcceleration3D> *> m_accOut;
    std::vector<RTC::OutPort<RTC::TimedAngularVelocity3D> *> m_rateOut;
    std::vector<RTC::OutPort<RTC::TimedDoubleSeq> *> m_forceOut;

    int dummy;
};

typedef boost::intrusive_ptr<BodyRTC> BodyRTCPtr;

#endif

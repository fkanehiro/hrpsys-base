#ifndef __OPENRTM_UTIL_H__
#define __OPENRTM_UTIL_H__

#include <rtm/RTObject.h>

int connectPorts(RTC::PortService_ptr outPort, RTC::PortService_ptr inPort);
void activateRtc(RTC::RtcBase* pRtc);
void deactivateRtc(RTC::RtcBase* pRtc);

class ClockReceiver
{
public:
    ClockReceiver(OpenRTM::ExtTrigExecutionContextService_ptr i_ec,
                  double i_period) : 
        m_ec(i_ec), m_period(i_period), m_time(i_period){}
    void tick(double dt){
        m_time += dt;
        if (m_time + dt/2 > m_period){
            m_ec->tick();
            m_time -= m_period;
        }
    }
private:
    OpenRTM::ExtTrigExecutionContextService_ptr m_ec;
    double m_period;
    double m_time; ///< duration since the last period
};


#endif

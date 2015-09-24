#include "hrpEC.h"
#include "io/iob.h"
#ifdef OPENRTM_VERSION_TRUNK
#include <rtm/RTObjectStateMachine.h>
#endif

#ifdef __QNX__
using std::fprintf;
#endif

namespace RTC
{
    hrpExecutionContext::~hrpExecutionContext()
    {
    }
    int hrpExecutionContext::svc(void)
    {
        if (open_iob() == FALSE){
            std::cerr << "open_iob: failed to open" << std::endl;
            return 0;
        } 
        if (lock_iob() == FALSE){
            std::cerr << "failed to lock iob" << std::endl;
            close_iob();
            return 0;
        }
#ifndef OPENRTM_VERSION_TRUNK
        long period_nsec = (m_period.sec()*1e9+m_period.usec()*1e3);
        double period_sec = period_nsec/1e9;
#else
        coil::TimeValue period(getPeriod());
        double period_sec = (double)period;
        long period_nsec = period_sec*1e9;
#endif
	    int nsubstep = number_of_substeps();
        set_signal_period(period_nsec/nsubstep);
        std::cout << "period = " << get_signal_period()*nsubstep/1e6
                  << "[ms], priority = " << m_priority << std::endl;

        if (!enterRT()){
            unlock_iob();
            close_iob();
            return 0;
        }
        do{
            if (!waitForNextPeriod()){
                unlock_iob();
                close_iob();
                return 0;
            }
            struct timeval tv;
            gettimeofday(&tv, NULL);
            if (m_profile.count > 0){
#define DELTA_SEC(start, end) end.tv_sec - start.tv_sec + (end.tv_usec - start.tv_usec)/1e6;
                double dt = DELTA_SEC(m_tv, tv);
                if (dt > m_profile.max_period) m_profile.max_period = dt;
                if (dt < m_profile.min_period) m_profile.min_period = dt;
                m_profile.avg_period = (m_profile.avg_period*m_profile.count + dt)/(m_profile.count+1);
            }
            m_profile.count++;
            m_tv = tv;

#ifndef OPENRTM_VERSION_TRUNK
            invoke_worker iw;
            struct timeval tbegin, tend;
	        std::vector<double> processes(m_comps.size());
            gettimeofday(&tbegin, NULL);
            for (unsigned int i=0; i< m_comps.size(); i++){
                iw(m_comps[i]);
                gettimeofday(&tend, NULL);
                double dt = DELTA_SEC(tbegin, tend);
                processes[i] = dt;
                tbegin = tend;
            }
#else
            struct timeval tbegin, tend;
            const RTCList& list = getComponentList();
            std::vector<double> processes(list.length());
            gettimeofday(&tbegin, NULL);
            for (unsigned int i=0; i< list.length(); i++){
                RTC_impl::RTObjectStateMachine* rtobj = m_worker.findComponent(list[i]);
                rtobj->workerDo(); 
                gettimeofday(&tend, NULL);
                double dt = DELTA_SEC(tbegin, tend);
                processes[i] = dt;
                tbegin = tend;
            }
#endif

            gettimeofday(&tv, NULL);
            double dt = DELTA_SEC(m_tv, tv);
            if (dt > m_profile.max_process) m_profile.max_process = dt;
	    if (m_profile.profiles.length() != processes.size()){
	        m_profile.profiles.length(processes.size());
		for (unsigned int i=0; i<m_profile.profiles.length(); i++){
		    m_profile.profiles[i].count = 0;
		    m_profile.profiles[i].avg_process = 0;
		    m_profile.profiles[i].max_process = 0;
		}
	    }
	    for (unsigned int i=0; i<m_profile.profiles.length(); i++){
#ifndef OPENRTM_VERSION_TRUNK
                LifeCycleState lcs = get_component_state(m_comps[i]._ref);
#else
                RTC_impl::RTObjectStateMachine* rtobj = m_worker.findComponent(list[i]);
                LifeCycleState lcs = rtobj->getState();
#endif
                OpenHRP::ExecutionProfileService::ComponentProfile &prof 
                    = m_profile.profiles[i];
                double dt = processes[i];
                if (lcs == ACTIVE_STATE){
                    prof.avg_process = (prof.avg_process*prof.count + dt)/(++prof.count);
                }
	        if (prof.max_process < dt) prof.max_process = dt;
	    }
            if (dt > period_sec*nsubstep){
  	        m_profile.timeover++; 
#ifdef NDEBUG
                fprintf(stderr, "[%d.%6.6d] Timeover: processing time = %4.2f[ms]\n",
                        tv.tv_sec, tv.tv_usec, dt*1e3);
                // Update rtc_names only when rtcs length change.
                if (processes.size() != rtc_names.size()){
                    rtc_names.clear();
                    for (unsigned int i=0; i< processes.size(); i++){
                        RTC::RTObject_var rtc = RTC::RTObject::_narrow(m_comps[i]._ref);
                        rtc_names.push_back(std::string(rtc->get_component_profile()->instance_name));
                    }
                }
                for (unsigned int i=0; i< processes.size(); i++){
                    fprintf(stderr, "%s(%4.2f), ", rtc_names[i].c_str(),processes[i]*1e3);
                }
                fprintf(stderr, "\n");
#endif
            }

#ifndef OPENRTM_VERSION_TRUNK
        } while (m_running);
#else
        } while (isRunning());
#endif
        exitRT();
        unlock_iob();
        close_iob();

        return 0;
    }

    OpenHRP::ExecutionProfileService::Profile *hrpExecutionContext::getProfile()
    {
        OpenHRP::ExecutionProfileService::Profile *ret 
            = new OpenHRP::ExecutionProfileService::Profile;
        *ret = m_profile;
        return ret;
    }

    OpenHRP::ExecutionProfileService::ComponentProfile hrpExecutionContext::getComponentProfile(RTC::LightweightRTObject_ptr obj)
    {
#ifndef OPENRTM_VERSION_TRUNK
        for (size_t i=0; i<m_comps.size(); i++){
            if (m_comps[i]._ref->_is_equivalent(obj)){
#else
        const RTCList& list = getComponentList();
        for(size_t i=0; i<list.length(); i++){
            RTC_impl::RTObjectStateMachine* rtobj = m_worker.findComponent(list[i]);
            if(rtobj->isEquivalent(obj)){
#endif
                return m_profile.profiles[i];
            }
        }
        throw OpenHRP::ExecutionProfileService::ExecutionProfileServiceException("no such component");
    }

    void hrpExecutionContext::resetProfile()
    {
        m_profile.max_period = m_profile.avg_period = 0;
        m_profile.min_period = 1.0; // enough long 
        m_profile.max_process = 0.0;
	for( unsigned int i = 0 ; i < m_profile.profiles.length() ; i++ ){
            m_profile.profiles[i].count       = 0;
	    m_profile.profiles[i].avg_process = 0;
	    m_profile.profiles[i].max_process = 0;
        }
        m_profile.count = m_profile.timeover = 0;
    }
};

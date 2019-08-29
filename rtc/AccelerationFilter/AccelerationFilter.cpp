// -*- C++ -*-
/*!
 * @file  AccelerationFilter.cpp * @brief Acceleration Filter component * $Date$ 
 *
 * $Id$ 
 */
#include "AccelerationFilter.h"

typedef coil::Guard<coil::Mutex> Guard;
// Module specification
// <rtc-template block="module_spec">
static const char* accelerationfilter_spec[] =
  {
    "implementation_id", "AccelerationFilter",
    "type_name",         "AccelerationFilter",
    "description",       "Acceleration Filter component",
    "version",           "1.0",
    "vendor",            "JSK",
    "category",          "example",
    "activity_type",     "SPORADIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    ""
  };
// </rtc-template>

AccelerationFilter::AccelerationFilter(RTC::Manager* manager)
    // <rtc-template block="initializer">
    : RTC::DataFlowComponentBase(manager),
      m_accInIn("accIn", m_accIn),
      m_rateInIn("rateIn", m_rateIn),
      m_rpyInIn("rpyIn", m_rpyIn),
      m_posInIn("posIn", m_posIn),
      m_velOutOut("velOut", m_velOut),
      //m_posOutOut("posOut", m_posOut),
      m_AccelerationFilterServicePort("AccelerationFilterService"),
    // </rtc-template>
      m_use_filter_bool(false)
{
}

AccelerationFilter::~AccelerationFilter()
{
}


RTC::ReturnCode_t AccelerationFilter::onInitialize()
{
    // Registration: InPort/OutPort/Service
    // <rtc-template block="registration">
    // Set InPort buffers
    addInPort("accIn", m_accInIn);
    addInPort("rateIn", m_rateInIn);
    addInPort("rpyIn", m_rpyInIn);
    addInPort("posIn", m_posInIn);

    // Set OutPort buffer
    addOutPort("velOut", m_velOutOut);
    //addOutPort("posOut", m_posOutOut);

    // Set service provider to Ports
    m_AccelerationFilterServicePort.registerProvider("service0", "AccelerationFilterService", m_service0);
    m_service0.setInstance(this);

    // Set service consumers to Ports

    // Set CORBA Service Ports
    addPort(m_AccelerationFilterServicePort);

    // </rtc-template>

    // <rtc-template block="bind_config">
    // Bind variables and configuration variable
    RTC::Properties& prop = getProperties();
    if ( ! coil::stringTo(m_dt, prop["dt"].c_str()) ) {
        std::cerr << "[" << m_profile.instance_name << "] failed to get dt" << std::endl;
        return RTC::RTC_ERROR;
    }

    // read gravity param
    double param_gravity = 9.80665;
    if ( ! coil::stringTo(m_gravity, prop["gravity"].c_str()) ) {
        param_gravity = m_gravity = 9.80665;
    }
    std::cerr << "[" << m_profile.instance_name << "] gravity : " << m_gravity << std::endl;

    // read filter param
    {
        coil::vstring filter_str = coil::split(prop["iir_filter_setting"], ",");
        if (filter_str.size() > 2) {
            int dim = (filter_str.size() - 1)/2;
            std::vector<double> bb;
            std::vector<double> aa;
            for(int i = 0; i < dim + 1; i++) {
                double val = -1;
                coil::stringTo(val, filter_str[i].c_str());
                bb.push_back(val);
            }
            for(int i = 0; i < filter_str.size() - dim - 1; i++) {
                double val = -1;
                coil::stringTo(val, filter_str[dim+1+i].c_str());
                aa.push_back(val);
            }
            if (aa.size() > 0 && bb.size() > 0) {
                m_use_filter_bool = true;
                std::cerr << "[" << m_profile.instance_name << "] pass filter_param : " << std::endl;
                std::cerr << "B = [";
                for(int i = 0; i < bb.size(); i++) {
                    std::cerr << " " << bb[i];
                }
                std::cerr << "]" << std::endl;
                std::cerr << "A = [";
                for(int i = 0; i < aa.size(); i++) {
                    std::cerr << " " << aa[i];
                }
                std::cerr << "]" << std::endl;
                for (int i = 0; i < 3; i++) {
                    IIRFilterPtr fl(new IIRFilter(std::string(m_profile.instance_name)));
                    fl->setParameter(dim, aa, bb);
                    fl->reset(param_gravity);
                    m_filters.push_back(fl);
                }
            }
        }
    }

    // </rtc-template>
    return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t AccelerationFilter::onFinalize()
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t AccelerationFilter::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t AccelerationFilter::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t AccelerationFilter::onActivated(RTC::UniqueId ec_id)
{
    std::cerr << "[" << m_profile.instance_name<< "] onActivated(" << ec_id << ")" << std::endl;
    return RTC::RTC_OK;
}
RTC::ReturnCode_t AccelerationFilter::onDeactivated(RTC::UniqueId ec_id)
{
    std::cerr << "[" << m_profile.instance_name<< "] onDeactivated(" << ec_id << ")" << std::endl;
    // reset filter
    return RTC::RTC_OK;
}


RTC::ReturnCode_t AccelerationFilter::onExecute(RTC::UniqueId ec_id)
{
    if (m_rpyInIn.isNew()) {
        m_rpyInIn.read();
    }
    if (m_rateInIn.isNew()) {
        m_rateInIn.read();
    }
    // calc expected velocity from AutoBalancer
    hrp::Vector3 expected_vel;
    if (m_posInIn.isNew()) {
        m_posInIn.read();
        hrp::Vector3 pos(m_posIn.data.x, m_posIn.data.y, m_posIn.data.z);
        expected_vel = pos - m_previous_pos;
        expected_vel /= m_dt;
        m_previous_pos = pos;
    }

    //
    if (m_accInIn.isNew()) {
        Guard guard(m_mutex);

        m_accInIn.read();
        hrp::Vector3 acc(m_accIn.data.ax, m_accIn.data.ay, m_accIn.data.az);
        hrp::Matrix33 imuR = hrp::rotFromRpy(m_rpyIn.data.r,
                                             m_rpyIn.data.p,
                                             m_rpyIn.data.y);
        hrp::Vector3 gravity(0, 0, - m_gravity);
        hrp::Vector3 acc_wo_g = imuR * acc + gravity;

        for (int i = 0; i < 3; i++) {
            if (m_use_filter_bool) {
                double filtered_acc =  m_filters[i]->passFilter(acc_wo_g[i]);
                m_global_vel[i] += filtered_acc * m_dt;
            } else {
                m_global_vel[i] += acc_wo_g[i] * m_dt;
            }
        }

        hrp::Vector3 _result_vel = imuR.inverse() * m_global_vel; // result should be described in sensor coords

        m_velOut.data.x  = _result_vel[0];
        m_velOut.data.y  = _result_vel[1];
        m_velOut.data.z  = _result_vel[2];
        m_velOutOut.write();
    }

    return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t AccelerationFilter::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t AccelerationFilter::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t AccelerationFilter::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t AccelerationFilter::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t AccelerationFilter::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

bool AccelerationFilter::resetFilter(const OpenHRP::AccelerationFilterService::ControlMode &mode,
                                     const double *vel)
{
    Guard guard(m_mutex);
    switch(mode) {
    case OpenHRP::AccelerationFilterService::MODE_ZERO_VELOCITY:
        m_global_vel[0] = 0;
        m_global_vel[1] = 0;
        m_global_vel[2] = 0;
        break;
    case OpenHRP::AccelerationFilterService::MODE_RELATIVE_GLOBAL_VELOCITY:
        m_global_vel[0] += vel[0];
        m_global_vel[1] += vel[1];
        m_global_vel[2] += vel[2];
        break;
    case OpenHRP::AccelerationFilterService::MODE_ABSOLUTE_GLOBAL_VELOCITY:
        m_global_vel[0] = vel[0];
        m_global_vel[1] = vel[1];
        m_global_vel[2] = vel[2];
        break;
    case OpenHRP::AccelerationFilterService::MODE_RELATIVE_LOCAL_VELOCITY:
        {
            hrp::Matrix33 imuR = hrp::rotFromRpy(m_rpyIn.data.r,
                                                 m_rpyIn.data.p,
                                                 m_rpyIn.data.y);
            hrp::Vector3 in_vel(vel[0], vel[1], vel[2]);
            hrp::Vector3 g_vel = imuR * in_vel;
            m_global_vel += g_vel;
        }
        break;
    case OpenHRP::AccelerationFilterService::MODE_ABSOLUTE_LOCAL_VELOCITY:
        {
            hrp::Matrix33 imuR = hrp::rotFromRpy(m_rpyIn.data.r,
                                                 m_rpyIn.data.p,
                                                 m_rpyIn.data.y);
            hrp::Vector3 in_vel(vel[0], vel[1], vel[2]);
            hrp::Vector3 g_vel = imuR * in_vel;
            m_global_vel = g_vel;
        }
        break;
    default:
        break;
    }
    return true;
}

bool AccelerationFilter::setParam(const ::OpenHRP::AccelerationFilterService::AccelerationFilterParam& i_param)
{
    Guard guard(m_mutex);
    m_gravity = i_param.gravity;

    if(i_param.filter_param.length() > 1) {
        int dim;
        std::vector<double> A;
        std::vector<double> B;
        dim = (i_param.filter_param.length() - 1)/2;
        for(int i = 0; i < dim + 1; i++) {
            B.push_back(i_param.filter_param[i]);
        }
        for(int i = 0; i < i_param.filter_param.length() - dim - 1; i++) {
            A.push_back(i_param.filter_param[dim+1+i]);
        }
        m_filters.resize(0);
        for(int i = 0; i < 3; i++) {
            IIRFilterPtr fl(new IIRFilter);
            fl->setParameter(dim, A, B);
            m_filters.push_back(fl);
        }
        m_use_filter_bool = i_param.use_filter;
    }
    return true;
}

bool AccelerationFilter::getParam(::OpenHRP::AccelerationFilterService::AccelerationFilterParam &i_param)
{
    i_param.gravity = m_gravity;
    i_param.use_filter = m_use_filter_bool;
    if(m_filters.size() > 0) {
        int dim;
        std::vector<double> A;
        std::vector<double> B;
        m_filters[0]->getParameter(dim, A, B);
        i_param.filter_param.length(2*(dim+1));
        for(int i = 0; i < dim+1; i++) {
            i_param.filter_param[i] = B[i];
            i_param.filter_param[i + dim + 1] = A[i];
        }
    }
    return true;
}

extern "C"
{
  void AccelerationFilterInit(RTC::Manager* manager)
  {
    coil::Properties profile(accelerationfilter_spec);
    manager->registerFactory(profile,
                             RTC::Create<AccelerationFilter>,
                             RTC::Delete<AccelerationFilter>);
  }
};




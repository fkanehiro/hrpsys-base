// -*- C++ -*-
/*!
 * @file  AccelerationFilter.h * @brief Acceleration Filter component * @date  $Date$ 
 *
 * $Id$ 
 */
#ifndef ACCELERATIONFILTER_H
#define ACCELERATIONFILTER_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
//
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
//
#include <hrpModel/Body.h>
//
#include <../TorqueFilter/IIRFilter.h>

// Service implementation headers
// <rtc-template block="service_impl_h">
#include "AccelerationFilterService_impl.h"

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;

class AccelerationFilter  : public RTC::DataFlowComponentBase
{
public:
    AccelerationFilter(RTC::Manager* manager);
    ~AccelerationFilter();

    // The initialize action (on CREATED->ALIVE transition)
    // formaer rtc_init_entry() 
    virtual RTC::ReturnCode_t onInitialize();

    // The finalize action (on ALIVE->END transition)
    // formaer rtc_exiting_entry()
    // virtual RTC::ReturnCode_t onFinalize();

    // The startup action when ExecutionContext startup
    // former rtc_starting_entry()
    // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

    // The shutdown action when ExecutionContext stop
    // former rtc_stopping_entry()
    // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

    // The activated action (Active state entry action)
    // former rtc_active_entry()
    virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

    // The deactivated action (Active state exit action)
    // former rtc_active_exit()
    virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

    // The execution action that is invoked periodically
    // former rtc_active_do()
    virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

    // The aborting action when main logic error occurred.
    // former rtc_aborting_entry()
    // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

    // The error action in ERROR state
    // former rtc_error_do()
    // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

    // The reset action that is invoked resetting
    // This is same but different the former rtc_init_entry()
    // virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);

    // The state update action that is invoked after onExecute() action
    // no corresponding operation exists in OpenRTm-aist-0.2.0
    // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

    // The action that is invoked when execution context's rate is changed
    // no corresponding operation exists in OpenRTm-aist-0.2.0
    // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);
    bool resetFilter(const OpenHRP::AccelerationFilterService::ControlMode &mode,
                     const double *vel);
    bool setParam(const ::OpenHRP::AccelerationFilterService::AccelerationFilterParam& i_param);
    bool getParam(::OpenHRP::AccelerationFilterService::AccelerationFilterParam &i_param);

protected:
    // Configuration variable declaration
    // <rtc-template block="config_declare">
    // </rtc-template>

    // DataInPort declaration
    // <rtc-template block="inport_declare">
    TimedAcceleration3D m_accIn;
    InPort<TimedAcceleration3D> m_accInIn;
    TimedAngularVelocity3D m_rateIn;
    InPort<TimedAngularVelocity3D> m_rateInIn;
    TimedOrientation3D m_rpyIn;
    InPort<TimedOrientation3D> m_rpyInIn;
    TimedPoint3D m_posIn;
    InPort<TimedPoint3D> m_posInIn;
    // </rtc-template>

    // DataOutPort declaration
    // <rtc-template block="outport_declare">
    TimedVector3D m_velOut;
    OutPort<TimedVector3D> m_velOutOut;
    //TimedPoint3D m_posOut;
    //OutPort<TimedPoint3D> m_posOutOut;

    // </rtc-template>

    // CORBA Port declaration
    // <rtc-template block="corbaport_declare">
    RTC::CorbaPort m_AccelerationFilterServicePort;

    // </rtc-template>

    // Service declaration
    // <rtc-template block="service_declare">
    AccelerationFilterService_impl m_service0;

    // </rtc-template>

    // Consumer declaration
    // <rtc-template block="consumer_declare">

    // </rtc-template>

private:
    typedef boost::shared_ptr< IIRFilter> IIRFilterPtr;
    double m_dt;
    double m_gravity;
    bool m_use_filter_bool;
    hrp::Vector3 m_global_vel;
    std::vector<IIRFilterPtr > m_filters;
    hrp::Vector3 m_previous_pos;

    coil::Mutex m_mutex;
};


extern "C"
{
    DLL_EXPORT void AccelerationFilterInit(RTC::Manager* manager);
};

#endif // ACCELERATIONFILTER_H


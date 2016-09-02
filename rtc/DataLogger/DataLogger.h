// -*- C++ -*-
/*!
 * @file  DataLogger.h
 * @brief logger component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include <deque>
#include <iomanip>

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>
#include "hrpsys/idl/HRPDataTypes.hh"
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>

// Service implementation headers
// <rtc-template block="service_impl_h">
#include "DataLoggerService_impl.h"

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;

#define DEFAULT_MAX_LOG_LENGTH (200*20)

class LoggerPortBase
{
public:
    LoggerPortBase() : m_maxLength(DEFAULT_MAX_LOG_LENGTH) {}
    virtual const char *name() = 0;
    virtual void clear() = 0;
    virtual void dumpLog(std::ostream& os) = 0;
    virtual void log() = 0;
    void maxLength(unsigned int len) { m_maxLength = len; }
protected:
    unsigned int m_maxLength;
};

/**
   \brief sample RT component which has one data input port and one data output port
 */
class DataLogger
  : public RTC::DataFlowComponentBase
{
 public:
  /**
     \brief Constructor
     \param manager pointer to the Manager
  */
  DataLogger(RTC::Manager* manager);
  /**
     \brief Destructor
  */
  virtual ~DataLogger();

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
  bool add(const char *i_type, const char *i_name);
  bool save(const char *i_basename);
  bool clear();
  void suspendLogging();
  void resumeLogging();
  void maxLength(unsigned int len);

  std::vector<LoggerPortBase *> m_ports;

 protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  
  // </rtc-template>

  TimedLong m_emergencySignal;

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  InPort<TimedLong> m_emergencySignalIn;
  
  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  RTC::CorbaPort m_DataLoggerServicePort;
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  DataLoggerService_impl m_service0;
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  
  // </rtc-template>

 private:
  bool m_suspendFlag;
  coil::Mutex m_suspendFlagMutex;
  int dummy;
};


extern "C"
{
  void DataLoggerInit(RTC::Manager* manager);
};

#endif // DATA_LOGGER_H

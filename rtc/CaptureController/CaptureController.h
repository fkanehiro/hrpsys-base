// -*- C++ -*-
/*!
 * @file  CaptureController.h
 * @brief capture controller
 * @date  $Date$
 *
 * $Id$
 */

#ifndef CAPTURE_CONTROLLER_H
#define CAPTURE_CONTROLLER_H

// Service implementation headers
// <rtc-template block="service_impl_h">
#include "CameraCaptureService_impl.h"

#include <rtm/idl/BasicDataType.hh>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;

/**
   \brief sample RT component which has one data input port and one data output port
 */
class CaptureController
  : public RTC::DataFlowComponentBase
{
 public:
  /**
     \brief Constructor
     \param manager pointer to the Manager
  */
  CaptureController(RTC::Manager* manager);
  /**
     \brief Destructor
  */
  virtual ~CaptureController();

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

  void take_one_frame();
  void start_continuous();
  void stop_continuous();

 protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  
  // </rtc-template>

  Img::TimedCameraImage m_image;

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  InPort<Img::TimedCameraImage> m_imageIn;
  
  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  OutPort<Img::TimedCameraImage> m_imageOut;
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  RTC::CorbaPort m_CameraCaptureServicePort;
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  CameraCaptureService_impl m_CameraCaptureService;
  
  // </rtc-template>

 private:
  typedef enum {SLEEP, ONESHOT, CONTINUOUS} mode;
  mode m_mode;
  int m_frameRate;
  double m_tOld;
  std::string m_initialMode;
  int dummy;
};


extern "C"
{
  void CaptureControllerInit(RTC::Manager* manager);
};

#endif // CAPTURE_CONTROLLER_H

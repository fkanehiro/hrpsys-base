// -*- C++ -*-
/*!
 * @file  OGMap3DViewer.h
 * @brief null component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef OGMAP3DVIEWER_H
#define OGMAP3DVIEWER_H

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/InterfaceDataTypes.hh>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/InterfaceDataTypes.hh>
//Open CV headder
#include <cv.h>
#include <highgui.h>
#include "hrpsys/idl/OGMap3DService.hh"

class GLbody;
class CMapSceneNode;

// Service implementation headers
// <rtc-template block="service_impl_h">

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;

/**
   \brief sample RT component which has one data input port and one data output port
 */
class OGMap3DViewer
  : public RTC::DataFlowComponentBase
{
 public:
  /**
     \brief Constructor
     \param manager pointer to the Manager
  */
  OGMap3DViewer(RTC::Manager* manager);
  /**
     \brief Destructor
  */
  virtual ~OGMap3DViewer();

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


 protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  
  // </rtc-template>

  OGMapCells m_cells;
  TimedDoubleSeq m_q;
  TimedPoint3D m_p;
  TimedOrientation3D m_rpy;

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  InPort<TimedDoubleSeq> m_qIn;
  InPort<TimedPoint3D> m_pIn;
  InPort<TimedOrientation3D> m_rpyIn;
  
  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  RTC::CorbaPort m_OGMap3DServicePort;
  RTC::CorbaConsumer<OpenHRP::OGMap3DService> m_OGMap3DService;
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  
  // </rtc-template>

 private:
  int dummy;
  bool m_isopen;
  double m_xSize, m_ySize, m_zSize;
  double m_xOrigin, m_yOrigin, m_zOrigin;
  bool m_generateImageSequence;
  GLbody *m_body;
  unsigned int m_imageCount;
  bool m_generateMovie, m_isGeneratingMovie;
  CMapSceneNode *m_mapNode;
  OpenHRP::OGMap3D *m_ogmap;
  CvVideoWriter *m_videoWriter;
  IplImage *m_cvImage;
};


extern "C"
{
  void OGMap3DViewerInit(RTC::Manager* manager);
};

#endif // OGMAP3DVIEWER_H

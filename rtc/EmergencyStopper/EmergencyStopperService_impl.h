#ifndef __EMERGENCYSTOPPER_SERVICE_H__
#define __EMERGENCYSTOPPER_SERVICE_H__

#include "hrpsys/idl/EmergencyStopperService.hh"

class EmergencyStopper;

class EmergencyStopperService_impl
    : public virtual POA_OpenHRP::EmergencyStopperService,
      public virtual PortableServer::RefCountServantBase {
 public:
  /**
     \brief constructor
  */
  EmergencyStopperService_impl();

  /**
     \brief destructor
  */
  virtual ~EmergencyStopperService_impl();

  void stopMotion();
  void releaseMotion();
  CORBA::Boolean getEmergencyStopperParam(
      OpenHRP::EmergencyStopperService::EmergencyStopperParam& i_param);
  CORBA::Boolean setEmergencyStopperParam(
      const OpenHRP::EmergencyStopperService::EmergencyStopperParam& i_param);

  void emergencystopper(EmergencyStopper* i_emergencystopper);

 private:
  EmergencyStopper* m_emergencystopper;
};

#endif

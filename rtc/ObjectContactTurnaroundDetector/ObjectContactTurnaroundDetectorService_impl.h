// -*-C++-*-
#ifndef OBJECTCONTACTTURNAROUNDDETECTORSERVICESVC_IMPL_H
#define OBJECTCONTACTTURNAROUNDDETECTORSERVICESVC_IMPL_H

#include "hrpsys/idl/ObjectContactTurnaroundDetectorService.hh"

using namespace OpenHRP;

class ObjectContactTurnaroundDetector;

class ObjectContactTurnaroundDetectorService_impl 
  : public virtual POA_OpenHRP::ObjectContactTurnaroundDetectorService,
    public virtual PortableServer::RefCountServantBase
{
public:
  ObjectContactTurnaroundDetectorService_impl();
  virtual ~ObjectContactTurnaroundDetectorService_impl();
  //
  void startObjectContactTurnaroundDetection(const CORBA::Double i_ref_diff_wrench, const CORBA::Double i_max_time, const OpenHRP::ObjectContactTurnaroundDetectorService::StrSequence& i_ee_names);
  OpenHRP::ObjectContactTurnaroundDetectorService::DetectorMode checkObjectContactTurnaroundDetection();
  CORBA::Boolean setObjectContactTurnaroundDetectorParam(const OpenHRP::ObjectContactTurnaroundDetectorService::objectContactTurnaroundDetectorParam &i_param_);
  CORBA::Boolean getObjectContactTurnaroundDetectorParam(OpenHRP::ObjectContactTurnaroundDetectorService::objectContactTurnaroundDetectorParam_out i_param_);
  CORBA::Boolean getObjectForcesMoments(OpenHRP::ObjectContactTurnaroundDetectorService::Dbl3Sequence_out o_forces, OpenHRP::ObjectContactTurnaroundDetectorService::Dbl3Sequence_out o_moments, OpenHRP::ObjectContactTurnaroundDetectorService::DblSequence3_out o_3dofwrench, CORBA::Double& o_fric_coeff_wrench);
  CORBA::Boolean checkObjectContactTurnaroundDetectionForGeneralizedWrench(OpenHRP::ObjectContactTurnaroundDetectorService::DetectorModeSequence_out o_dms);
  CORBA::Boolean startObjectContactTurnaroundDetectionForGeneralizedWrench();
  CORBA::Boolean getObjectGeneralizedConstraintWrenches(OpenHRP::ObjectContactTurnaroundDetectorService::objectGeneralizedConstraintWrenchesParam_out o_param);

  //
  void octd(ObjectContactTurnaroundDetector *i_octd);
private:
  ObjectContactTurnaroundDetector *m_octd;
};

#endif // OBJECTCONTACTTURNAROUNDDETECTORSERVICESVC_IMPL_H

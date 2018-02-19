#include "ObjectContactTurnaroundDetectorService_impl.h"
#include "ObjectContactTurnaroundDetector.h"
#include <hrpModel/Body.h>
#include <hrpModel/Link.h>

ObjectContactTurnaroundDetectorService_impl::ObjectContactTurnaroundDetectorService_impl() : m_octd(NULL)
{
}

ObjectContactTurnaroundDetectorService_impl::~ObjectContactTurnaroundDetectorService_impl()
{
}

void ObjectContactTurnaroundDetectorService_impl::startObjectContactTurnaroundDetection(const CORBA::Double i_ref_diff_wrench, const CORBA::Double i_max_time, const OpenHRP::ObjectContactTurnaroundDetectorService::StrSequence& i_ee_names)
{
  m_octd->startObjectContactTurnaroundDetection(i_ref_diff_wrench, i_max_time, i_ee_names);
}

OpenHRP::ObjectContactTurnaroundDetectorService::DetectorMode ObjectContactTurnaroundDetectorService_impl::checkObjectContactTurnaroundDetection()
{
  return m_octd->checkObjectContactTurnaroundDetection();
}

CORBA::Boolean ObjectContactTurnaroundDetectorService_impl::setObjectContactTurnaroundDetectorParam(const OpenHRP::ObjectContactTurnaroundDetectorService::objectContactTurnaroundDetectorParam &i_param_)
{
  return m_octd->setObjectContactTurnaroundDetectorParam(i_param_);
}

CORBA::Boolean ObjectContactTurnaroundDetectorService_impl::getObjectContactTurnaroundDetectorParam(OpenHRP::ObjectContactTurnaroundDetectorService::objectContactTurnaroundDetectorParam& i_param_)
{
  i_param_ = OpenHRP::ObjectContactTurnaroundDetectorService::objectContactTurnaroundDetectorParam();
  return m_octd->getObjectContactTurnaroundDetectorParam(i_param_);
}

CORBA::Boolean ObjectContactTurnaroundDetectorService_impl::getObjectForcesMoments(OpenHRP::ObjectContactTurnaroundDetectorService::Dbl3Sequence_out o_forces, OpenHRP::ObjectContactTurnaroundDetectorService::Dbl3Sequence_out o_moments, OpenHRP::ObjectContactTurnaroundDetectorService::DblSequence3_out o_3dofwrench, CORBA::Double& o_fric_coeff_wrench)
{
  return m_octd->getObjectForcesMoments(o_forces, o_moments, o_3dofwrench, o_fric_coeff_wrench);
}

CORBA::Boolean ObjectContactTurnaroundDetectorService_impl::getObjectGeneralizedConstraintWrenches(CORBA::Double& o_generalized_constraint_wrench1, CORBA::Double& o_generalized_constraint_wrench2)
{
  return m_octd->getObjectGeneralizedConstraintWrenches(o_generalized_constraint_wrench1, o_generalized_constraint_wrench2);
}

void ObjectContactTurnaroundDetectorService_impl::octd(ObjectContactTurnaroundDetector *i_octd)
{
  m_octd = i_octd;
} 


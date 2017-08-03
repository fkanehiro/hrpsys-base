#include "ObjectContactTurnaroundDetectorService_impl.h"
#include "ObjectContactTurnaroundDetector.h"
#include <hrpModel/Body.h>
#include <hrpModel/Link.h>

ObjectContactTurnaroundDetectorService_impl::ObjectContactTurnaroundDetectorService_impl() : m_otd(NULL)
{
}

ObjectContactTurnaroundDetectorService_impl::~ObjectContactTurnaroundDetectorService_impl()
{
}

void ObjectContactTurnaroundDetectorService_impl::startObjectContactTurnaroundDetection(const CORBA::Double i_ref_diff_wrench, const CORBA::Double i_max_time, const OpenHRP::ObjectContactTurnaroundDetectorService::StrSequence& i_ee_names)
{
  m_otd->startObjectContactTurnaroundDetection(i_ref_diff_wrench, i_max_time, i_ee_names);
}

OpenHRP::ObjectContactTurnaroundDetectorService::DetectorMode ObjectContactTurnaroundDetectorService_impl::checkObjectContactTurnaroundDetection()
{
  return m_otd->checkObjectContactTurnaroundDetection();
}

CORBA::Boolean ObjectContactTurnaroundDetectorService_impl::setObjectContactTurnaroundDetectorParam(const OpenHRP::ObjectContactTurnaroundDetectorService::objectContactTurnaroundDetectorParam &i_param_)
{
  return m_otd->setObjectContactTurnaroundDetectorParam(i_param_);
}

CORBA::Boolean ObjectContactTurnaroundDetectorService_impl::getObjectContactTurnaroundDetectorParam(OpenHRP::ObjectContactTurnaroundDetectorService::objectContactTurnaroundDetectorParam& i_param_)
{
  i_param_ = OpenHRP::ObjectContactTurnaroundDetectorService::objectContactTurnaroundDetectorParam();
  return m_otd->getObjectContactTurnaroundDetectorParam(i_param_);
}

CORBA::Boolean ObjectContactTurnaroundDetectorService_impl::getObjectForcesMoments(OpenHRP::ObjectContactTurnaroundDetectorService::Dbl3Sequence_out o_forces, OpenHRP::ObjectContactTurnaroundDetectorService::Dbl3Sequence_out o_moments, OpenHRP::ObjectContactTurnaroundDetectorService::DblSequence3_out o_3dofwrench, CORBA::Double& o_fric_coeff_wrench)
{
  return m_otd->getObjectForcesMoments(o_forces, o_moments, o_3dofwrench, o_fric_coeff_wrench);
}

void ObjectContactTurnaroundDetectorService_impl::otd(ObjectContactTurnaroundDetector *i_otd)
{
  m_otd = i_otd;
} 


#include "ImpedanceControllerService_impl.h"
#include "ImpedanceController.h"
#include <hrpModel/Body.h>
#include <hrpModel/Link.h>

ImpedanceControllerService_impl::ImpedanceControllerService_impl() : m_impedance(NULL)
{
}

ImpedanceControllerService_impl::~ImpedanceControllerService_impl()
{
}

CORBA::Boolean ImpedanceControllerService_impl::startImpedanceController(const char *i_name_)
{
  return m_impedance->startImpedanceController(std::string(i_name_));
}

CORBA::Boolean ImpedanceControllerService_impl::startImpedanceControllerNoWait(const char *i_name_)
{
  return m_impedance->startImpedanceControllerNoWait(std::string(i_name_));
}

CORBA::Boolean ImpedanceControllerService_impl::stopImpedanceController(const char *i_name_)
{
  return m_impedance->stopImpedanceController(std::string(i_name_));
}

CORBA::Boolean ImpedanceControllerService_impl::stopImpedanceControllerNoWait(const char *i_name_)
{
  return m_impedance->stopImpedanceControllerNoWait(std::string(i_name_));
}

CORBA::Boolean ImpedanceControllerService_impl::setImpedanceControllerParam(const char *i_name_, const OpenHRP::ImpedanceControllerService::impedanceParam &i_param_)
{
  return m_impedance->setImpedanceControllerParam(std::string(i_name_), i_param_);
}

CORBA::Boolean ImpedanceControllerService_impl::getImpedanceControllerParam(const char *i_name_, OpenHRP::ImpedanceControllerService::impedanceParam_out i_param_)
{
  i_param_ = new OpenHRP::ImpedanceControllerService::impedanceParam();
  i_param_->force_gain.length(3);
  i_param_->moment_gain.length(3);
  return m_impedance->getImpedanceControllerParam(std::string(i_name_), *i_param_);
}

void ImpedanceControllerService_impl::waitImpedanceControllerTransition(const char *i_name_)
{
  m_impedance->waitImpedanceControllerTransition(std::string(i_name_));
}

void ImpedanceControllerService_impl::startObjectTurnaroundDetection(const CORBA::Double i_ref_diff_wrench, const CORBA::Double i_max_time, const OpenHRP::ImpedanceControllerService::StrSequence& i_ee_names)
{
  m_impedance->startObjectTurnaroundDetection(i_ref_diff_wrench, i_max_time, i_ee_names);
}

CORBA::Boolean ImpedanceControllerService_impl::checkObjectTurnaroundDetection()
{
  return m_impedance->checkObjectTurnaroundDetection();
}

CORBA::Boolean ImpedanceControllerService_impl::setObjectTurnaroundDetectorParam(const OpenHRP::ImpedanceControllerService::objectTurnaroundDetectorParam &i_param_)
{
  return m_impedance->setObjectTurnaroundDetectorParam(i_param_);
}

CORBA::Boolean ImpedanceControllerService_impl::getObjectTurnaroundDetectorParam(OpenHRP::ImpedanceControllerService::objectTurnaroundDetectorParam& i_param_)
{
  i_param_ = OpenHRP::ImpedanceControllerService::objectTurnaroundDetectorParam();
  return m_impedance->getObjectTurnaroundDetectorParam(i_param_);
}

CORBA::Boolean ImpedanceControllerService_impl::getObjectForcesMoments(OpenHRP::ImpedanceControllerService::Dbl3Sequence_out o_forces, OpenHRP::ImpedanceControllerService::Dbl3Sequence_out o_moments)
{
  return m_impedance->getObjectForcesMoments(o_forces, o_moments);
}

void ImpedanceControllerService_impl::impedance(ImpedanceController *i_impedance)
{
  m_impedance = i_impedance;
} 


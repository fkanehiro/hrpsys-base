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

CORBA::Boolean ImpedanceControllerService_impl::setImpedanceControllerParam(const OpenHRP::ImpedanceControllerService::impedanceParam &i_param_)
{
  return m_impedance->setImpedanceControllerParam(i_param_);
}

CORBA::Boolean ImpedanceControllerService_impl::getImpedanceControllerParam(const char *i_name_, OpenHRP::ImpedanceControllerService::impedanceParam_out i_param_)
{
  i_param_ = new OpenHRP::ImpedanceControllerService::impedanceParam();
  i_param_->ref_force.length(3);
  i_param_->ref_moment.length(3);
  i_param_->force_gain.length(3);
  i_param_->moment_gain.length(3);
  return m_impedance->getImpedanceControllerParam(std::string(i_name_), *i_param_);
}

CORBA::Boolean ImpedanceControllerService_impl::deleteImpedanceController(const char *i_name_)
{
  return m_impedance->deleteImpedanceController(std::string(i_name_));
}

void ImpedanceControllerService_impl::waitDeletingImpedanceController(const char *i_name_)
{
  m_impedance->waitDeletingImpedanceController(std::string(i_name_));
}

CORBA::Boolean ImpedanceControllerService_impl::deleteImpedanceControllerAndWait(const char *i_name_)
{
  return m_impedance->deleteImpedanceControllerAndWait(std::string(i_name_));
}

void ImpedanceControllerService_impl::impedance(ImpedanceController *i_impedance)
{
  m_impedance = i_impedance;
} 


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

CORBA::Boolean ImpedanceControllerService_impl::deleteImpedanceController(const char *i_name_)
{
  return m_impedance->deleteImpedanceController(std::string(i_name_));
}

void ImpedanceControllerService_impl::impedance(ImpedanceController *i_impedance)
{
  m_impedance = i_impedance;
} 


#include "WholeBodyMasterSlaveService_impl.h"
#include "WholeBodyMasterSlave.h"

WholeBodyMasterSlaveService_impl::WholeBodyMasterSlaveService_impl() : m_wholebodymasterslave(NULL)
{
}

WholeBodyMasterSlaveService_impl::~WholeBodyMasterSlaveService_impl()
{
}

void WholeBodyMasterSlaveService_impl::wholebodymasterslave(WholeBodyMasterSlave *i_wholebodymasterslave)
{
  m_wholebodymasterslave = i_wholebodymasterslave;
}

CORBA::Boolean WholeBodyMasterSlaveService_impl::startWholeBodyMasterSlave()
{
    return m_wholebodymasterslave->startWholeBodyMasterSlave();
};

CORBA::Boolean WholeBodyMasterSlaveService_impl::stopWholeBodyMasterSlave()
{
    return m_wholebodymasterslave->stopWholeBodyMasterSlave();
};

CORBA::Boolean WholeBodyMasterSlaveService_impl::pauseWholeBodyMasterSlave()
{
    return m_wholebodymasterslave->pauseWholeBodyMasterSlave();
};

CORBA::Boolean WholeBodyMasterSlaveService_impl::resumeWholeBodyMasterSlave()
{
    return m_wholebodymasterslave->resumeWholeBodyMasterSlave();
};

void WholeBodyMasterSlaveService_impl::setParams(const OpenHRP::WholeBodyMasterSlaveService::WholeBodyMasterSlaveParam& i_param)
{
  m_wholebodymasterslave->setParams(i_param);
};

void WholeBodyMasterSlaveService_impl::getParams(OpenHRP::WholeBodyMasterSlaveService::WholeBodyMasterSlaveParam_out i_param)
{
  i_param = new OpenHRP::WholeBodyMasterSlaveService::WholeBodyMasterSlaveParam();
  m_wholebodymasterslave->getParams(*i_param);
//  m_wholebodymasterslave->getParams(i_param); // error: no matching function for call to ‘WholeBodyMasterSlave::getParams(OpenHRP::WholeBodyMasterSlaveService::WholeBodyMasterSlaveParam_out&)’
};

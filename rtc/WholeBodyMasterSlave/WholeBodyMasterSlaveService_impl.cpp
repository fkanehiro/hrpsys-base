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

CORBA::Boolean WholeBodyMasterSlaveService_impl::startCountDownForWholeBodyMasterSlave(CORBA::Double sec)
{
    return m_wholebodymasterslave->startCountDownForWholeBodyMasterSlave(sec);
};

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

void WholeBodyMasterSlaveService_impl::setWholeBodyMasterSlaveParam(const OpenHRP::WholeBodyMasterSlaveService::WholeBodyMasterSlaveParam& i_param)
{
  m_wholebodymasterslave->setWholeBodyMasterSlaveParam(i_param);
};

void WholeBodyMasterSlaveService_impl::getWholeBodyMasterSlaveParam(OpenHRP::WholeBodyMasterSlaveService::WholeBodyMasterSlaveParam_out i_param)
{
//  i_param = new OpenHRP::WholeBodyMasterSlaveParam::WholeBodyMasterSlaveParam();
//  return  m_wholebodymasterslave->getWholeBodyMasterSlaveParam(*i_param);
  m_wholebodymasterslave->getWholeBodyMasterSlaveParam(i_param);
};

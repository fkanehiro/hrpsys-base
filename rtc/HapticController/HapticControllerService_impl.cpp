#include "HapticControllerService_impl.h"
#include "HapticController.h"

HapticControllerService_impl::HapticControllerService_impl() : m_hapticcontroller(NULL)
{
}

HapticControllerService_impl::~HapticControllerService_impl()
{
}

void HapticControllerService_impl::hapticcontroller(HapticController *i_hapticcontroller)
{
  m_hapticcontroller = i_hapticcontroller;
}

CORBA::Boolean HapticControllerService_impl::startHapticController()
{
    return m_hapticcontroller->startHapticController();
};

CORBA::Boolean HapticControllerService_impl::stopHapticController()
{
    return m_hapticcontroller->stopHapticController();
};

CORBA::Boolean HapticControllerService_impl::pauseHapticController()
{
    return m_hapticcontroller->pauseHapticController();
};

CORBA::Boolean HapticControllerService_impl::resumeHapticController()
{
    return m_hapticcontroller->resumeHapticController();
};
void HapticControllerService_impl::resetOdom()
{
  m_hapticcontroller->resetOdom();
};
//
void HapticControllerService_impl::setParams(const OpenHRP::HapticControllerService::HapticControllerParam& i_param)
{
  m_hapticcontroller->setParams(i_param);
};

void HapticControllerService_impl::getParams(OpenHRP::HapticControllerService::HapticControllerParam_out i_param)
{
    //????????????????????????????????????????????????????????
//    OpenHRP::HapticControllerService::HapticControllerParam tmp;
//    m_hapticcontroller->getParams(tmp);
//    i_param = tmp;

    //今までこれでいけてた？ idlにsequence<double, 2>とか可変長を含むとき？
    i_param = new OpenHRP::HapticControllerService::HapticControllerParam();
    m_hapticcontroller->getParams(*i_param);

    // error: no matching function for call to ‘HapticController::getParams(OpenHRP::HapticControllerService::HapticControllerParam_out&)’
//  m_hapticcontroller->getParams(i_param);
 };

#include "OnlineViewer_impl.h"
#include "GLmodel.h"
#include <iostream>
#include <hrpModel/ModelLoaderUtil.h>
#include <cstdio>

using namespace OpenHRP;


#if 0
    if (event&ADD_BODY){
        GLbody *body = new GLbody(binfo);
        scene->addBody(name, body);
        event &= ~ADD_BODY;
        sem_post(&sem);
    }
#endif
    
OnlineViewer_impl::OnlineViewer_impl(CORBA::ORB_ptr orb, PortableServer::POA_ptr poa)
    :
    orb(CORBA::ORB::_duplicate(orb)),
    poa(PortableServer::POA::_duplicate(poa)),
    scene(GLscene::getInstance())
{
}

OnlineViewer_impl::~OnlineViewer_impl()
{
}
		
PortableServer::POA_ptr OnlineViewer_impl::_default_POA()
{
    return PortableServer::POA::_duplicate(poa);
}
		
void OnlineViewer_impl::update(const WorldState& state)
{
    scene->addState(state);
}

void OnlineViewer_impl::load(const char* name_, const char* url)
{
    if (!scene->findBody(name_)){
        std::cout << "load(" << url << ")" << std::endl;
        BodyInfo_var binfo = hrp::loadBodyInfo(url, orb);
        scene->addBody(name_, binfo);
    }
}

void OnlineViewer_impl::clearLog()
{
    scene->clearLog();
}

void OnlineViewer_impl::clearData()
{
}

void OnlineViewer_impl::drawScene(const WorldState& state)
{
}

void OnlineViewer_impl::setLineWidth(::CORBA::Float width)
{
}

void OnlineViewer_impl::setLineScale(::CORBA::Float scale)
{
}

::CORBA::Boolean OnlineViewer_impl::getPosture(const char* robotId, DblSequence_out posture)
{
    return true;
}

void OnlineViewer_impl::setLogName(const char* name)
{
}


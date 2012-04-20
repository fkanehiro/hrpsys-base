#include <cstdio>
#include <iostream>
#include <hrpModel/ModelLoaderUtil.h>
#include "OnlineViewer_impl.h"
#include "GLscene.h"

using namespace OpenHRP;

OnlineViewer_impl::OnlineViewer_impl(CORBA::ORB_ptr orb, PortableServer::POA_ptr poa, GLscene *i_scene, LogManager<OpenHRP::WorldState> *i_log)
    :
    orb(CORBA::ORB::_duplicate(orb)),
    poa(PortableServer::POA::_duplicate(poa)),
    scene(i_scene), log(i_log)
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
    log->add(state);
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
    log->clear();
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


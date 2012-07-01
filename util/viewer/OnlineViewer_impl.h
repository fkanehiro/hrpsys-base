#include <hrpCorba/OnlineViewer.hh>
#include <map>
#include <string>
#include "util/LogManager.h"

class GLscene;
class GLbody;

namespace OpenHRP{

class OnlineViewer_impl : public POA_OpenHRP::OnlineViewer
{
public:
    OnlineViewer_impl(CORBA::ORB_ptr orb, PortableServer::POA_ptr poa,
                      GLscene *i_scene, LogManager<OpenHRP::WorldState> *i_log);
    virtual ~OnlineViewer_impl();
		
    virtual PortableServer::POA_ptr _default_POA();
		
    void update(const WorldState& state);
    void load(const char* name, const char* url);
    void clearLog();
    void clearData();
    void drawScene(const WorldState& state);
    void setLineWidth(::CORBA::Float width);
    void setLineScale(::CORBA::Float scale);
    ::CORBA::Boolean getPosture(const char* robotId, DblSequence_out posture);
    void setLogName(const char* name);

private:
    CORBA::ORB_var orb;
    PortableServer::POA_var poa;
    GLscene *scene;
    LogManager<OpenHRP::WorldState> *log;
    std::map<std::string, GLbody *> models;
};

};

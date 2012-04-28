#include <fstream>
#include <iostream>
#include <hrpModel/ModelLoaderUtil.h>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include "util/Project.h"
#include "util/GLbody.h"
#include "util/SDLUtil.h"
#include "GLscene.h"
#include "Monitor.h"

using namespace hrp;
using namespace OpenHRP;

int main(int argc, char* argv[]) 
{
    if (argc < 2){
        std::cerr << "Usage:" << argv[0] << " project.xml" << std::endl;
        return 1;
    }

    Project prj;
    if (!prj.parse(argv[1])){
        std::cerr << "failed to parse " << argv[1] << std::endl;
        return 1;
    }

    //================= CORBA =========================
    CORBA::ORB_var orb;
    CosNaming::NamingContext_var namingContext;
 
    try {
	orb = CORBA::ORB_init(argc, argv);

	CORBA::Object_var obj;
	obj = orb->resolve_initial_references("RootPOA");
	PortableServer::POA_var poa = PortableServer::POA::_narrow(obj);
	if(CORBA::is_nil(poa)){
	    throw std::string("error: failed to narrow root POA.");
	}
	
	PortableServer::POAManager_var poaManager = poa->the_POAManager();
	if(CORBA::is_nil(poaManager)){
	    throw std::string("error: failed to narrow root POA manager.");
	}
	
	obj = orb->resolve_initial_references("NameService");
	namingContext = CosNaming::NamingContext::_narrow(obj);
	if(CORBA::is_nil(namingContext)){
	    throw std::string("error: failed to narrow naming context.");
	}
	
	poaManager->activate();
    }catch (CORBA::SystemException& ex) {
        std::cerr << ex._rep_id() << std::endl;
    }catch (const std::string& error){
        std::cerr << error << std::endl;
    }

    //================= logger ======================
    LogManager<TimedRobotState> log; 
    log.enableRingBuffer(5000);

    //================= monitor ======================
    Monitor monitor(orb, 
                    prj.robotHost(), prj.robotPort(), prj.interval(),
                    &log);
    //==================== viewer ===============
    GLscene scene(&log);

    SDLwindow window(&scene, &log, &monitor);
    window.init();

    ModelLoader_var modelloader = getModelLoader(namingContext);
    for (std::map<std::string, ModelItem>::iterator it=prj.models().begin();
         it != prj.models().end(); it++){
        OpenHRP::ModelLoader::ModelLoadOption opt;
        opt.readImage = true;
        opt.AABBdata.length(0);
        opt.AABBtype = OpenHRP::ModelLoader::AABB_NUM;
        OpenHRP::BodyInfo_var binfo
            = modelloader->getBodyInfoEx(it->second.url.c_str(), opt);
        GLbody *body = new GLbody(binfo);
        scene.addBody(it->first, body);
    }

    monitor.start();
    int cnt=0;
    while(window.oneStep());
    monitor.stop();

    try {
	orb->destroy();
    }
    catch(...){

    }

    return 0;
}

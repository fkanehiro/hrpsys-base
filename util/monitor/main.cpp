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
#include "util/GLlink.h"
#include "util/GLutil.h"
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

    char *rhname = NULL, *shname = NULL;
    for (int i = 2; i<argc; i++){
        if (strcmp(argv[i], "-rh")==0){
            rhname = argv[++i];
        }else if(strcmp(argv[i], "-sh")==0){
            shname = argv[++i];
        }
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
    RobotHardwareClientView rhview = prj.RobotHardwareClient();
    Monitor monitor(orb, 
                    rhview.hostname, rhview.port, rhview.interval,
                    &log);
    if (rhname) monitor.setRobotHardwareName(rhname);
    if (shname) monitor.setStateHolderName(shname);
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
        GLbody *glbody = new GLbody();
        hrp::BodyPtr body(glbody);
        hrp::loadBodyFromBodyInfo(body, binfo, false, GLlinkFactory);
        loadShapeFromBodyInfo(glbody, binfo);
        body->setName(it->first);
        scene.WorldBase::addBody(body);
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

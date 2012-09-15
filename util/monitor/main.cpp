#include <fstream>
#include <iostream>
#include <boost/bind.hpp>
#include <hrpModel/ModelLoaderUtil.h>
#include <hrpModel/ColdetLinkPair.h>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include "util/ProjectUtil.h"
#include "util/GLbody.h"
#include "util/GLlink.h"
#include "util/GLutil.h"
#include "util/SDLUtil.h"
#include "GLscene.h"
#include "Monitor.h"

using namespace hrp;
using namespace OpenHRP;

hrp::BodyPtr createBody(const std::string& name, const ModelItem& mitem,
                        ModelLoader_ptr modelloader)
{
    BodyInfo_var binfo;
    try{
        OpenHRP::ModelLoader::ModelLoadOption opt;
        opt.readImage = true;
        opt.AABBdata.length(0);
        opt.AABBtype = OpenHRP::ModelLoader::AABB_NUM;
        binfo = modelloader->getBodyInfoEx(mitem.url.c_str(), opt);
    }catch(OpenHRP::ModelLoader::ModelLoaderException ex){
        std::cerr << ex.description << std::endl;
        return hrp::BodyPtr();
    }
    GLbody *glbody = new GLbody();
    hrp::BodyPtr body(glbody);
    hrp::loadBodyFromBodyInfo(body, binfo, true, GLlinkFactory);
    loadShapeFromBodyInfo(glbody, binfo);
    body->setName(name);
    return body;
}

int main(int argc, char* argv[]) 
{
    if (argc < 2){
        std::cerr << "Usage:" << argv[0] << " project.xml [-rh RobotHardwareComponent] [-sh StateHolder component] [-size size] [-bg r g b]" << std::endl;
        return 1;
    }

    Project prj;
    if (!prj.parse(argv[1])){
        std::cerr << "failed to parse " << argv[1] << std::endl;
        return 1;
    }

    char *rhname = NULL, *shname = NULL;
    int wsize = 0;
    float bgColor[] = {0,0,0};
    for (int i = 2; i<argc; i++){
        if (strcmp(argv[i], "-rh")==0){
            rhname = argv[++i];
        }else if(strcmp(argv[i], "-sh")==0){
            shname = argv[++i];
        }else if(strcmp(argv[i], "-size")==0){
            wsize = atoi(argv[++i]);
        }else if(strcmp(argv[i], "-bg")==0){
            bgColor[0] = atof(argv[++i]);
            bgColor[1] = atof(argv[++i]);
            bgColor[2] = atof(argv[++i]);
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
    if (rhname) {
        monitor.setRobotHardwareName(rhname);
    }else{
        monitor.setRobotHardwareName(rhview.RobotHardwareName.c_str());
    }
    if (shname) {
        monitor.setStateHolderName(shname);
    }else{
        monitor.setStateHolderName(rhview.StateHolderName.c_str());
    }
    //==================== viewer ===============
    GLscene scene(&log);
    scene.setBackGroundColor(bgColor);
    scene.showCoMonFloor(prj.view().showCoMonFloor);

    SDLwindow window(&scene, &log, &monitor);
    window.init(wsize, wsize);

    std::vector<hrp::ColdetLinkPairPtr> pairs;
    ModelLoader_var modelloader = getModelLoader(namingContext);
    if (CORBA::is_nil(modelloader)){
        std::cerr << "openhrp-model-loader is not running" << std::endl;
        return 1;
    }
    BodyFactory factory = boost::bind(createBody, _1, _2, modelloader);
    initWorld(prj, factory, scene, pairs);
    scene.setCollisionCheckPairs(pairs);

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

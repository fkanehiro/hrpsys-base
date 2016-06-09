#include <fstream>
#include <iostream>
#include <boost/bind.hpp>
#include <hrpModel/ModelLoaderUtil.h>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include "hrpsys/util/ProjectUtil.h"
#include "hrpsys/util/GLbody.h"
#include "hrpsys/util/GLlink.h"
#include "hrpsys/util/GLutil.h"
#include "hrpsys/util/SDLUtil.h"
#include "hrpsys/util/LogManager.h"
#include "hrpsys/util/BVutil.h"
#include "TimedPosture.h"
#include "GLscene.h"
#include "hrpsys/idl/CollisionDetectorService.hh"

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

    convertToConvexHull(body);

    body->setName(name);
    return body;
}

int main(int argc, char* argv[])
{
    if (argc < 2){
        std::cerr << "Usage:" << argv[0] << " project.xml [-co CollisionDetector Component] [-size size] [-bg r g b]" << std::endl;
        return 1;
    }

    Project prj;
    if (!prj.parse(argv[1])){
        std::cerr << "failed to parse " << argv[1] << std::endl;
        return 1;
    }

    const char *coname = "co";
    int wsize = 0;
    float bgColor[] = {0,0,0};
    for (int i = 2; i<argc; i++){
        if (strcmp(argv[i], "-co")==0){
            coname = argv[++i];
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

    //==================== viewer ===============
    LogManager<OpenHRP::CollisionDetectorService::CollisionState> log;
    CollisionDetectorComponent::GLscene scene(&log);
    scene.setBackGroundColor(bgColor);

    SDLwindow window(&scene, &log);
    window.init(wsize, wsize);

    std::vector<hrp::ColdetLinkPairPtr> pairs;
    ModelLoader_var modelloader = getModelLoader(namingContext);
    if (CORBA::is_nil(modelloader)){
        std::cerr << "openhrp-model-loader is not running" << std::endl;
        return 1;
    }
    BodyFactory factory = boost::bind(createBody, _1, _2, modelloader);
    //hrp::BodyPtr m_robot = createBody("pa10", , modelloader);
    //scene.addBody(m_robot);
    initWorld(prj, factory, scene, pairs);
    GLlink::drawMode(GLlink::DM_COLLISION);
    scene.showFloorGrid(false);

    log.enableRingBuffer(1);
    OpenHRP::CollisionDetectorService_var coService;

    while(window.oneStep()) {
        //==================== collision detecter ===============

        if (CORBA::is_nil(coService)){
            try{
                CosNaming::Name name;
                name.length(1);
                name[0].id = CORBA::string_dup(coname);
                name[0].kind = CORBA::string_dup("rtc");
                CORBA::Object_var obj = namingContext->resolve(name);
                RTC::RTObject_var rtc = RTC::RTObject::_narrow(obj);
                const char *ior = getServiceIOR(rtc, "CollisionDetectorService");
                coService = OpenHRP::CollisionDetectorService::_narrow(orb->string_to_object(ior));
            }catch(...){
                std::cerr << "could not found collision detector component " << coname << std::endl;
                return 1;
            }
        }
        // get CollisionState
        OpenHRP::CollisionDetectorService::CollisionState co;
        bool stateUpdate = false;
        if (!CORBA::is_nil(coService)){
            try{
                OpenHRP::CollisionDetectorService::CollisionState_var t_co;
                coService->getCollisionStatus(t_co);
                co = t_co;
                stateUpdate = true;
            }catch(...){
                std::cerr << "exception in getCollisionStatus()" << std::endl;
                coService = NULL;
            }
        }

        if (stateUpdate) {
            try {
                GLbody* glbody = dynamic_cast<GLbody *>(scene.body(0).get());
                for (size_t i=0; i<co.collide.length(); i++) {
                    ((GLlink *)glbody->link(i))->highlight(co.collide[i]);
                }
#if 0
                TimedPosture tp;
                tp.time = co.time;
                tp.posture.resize(co.angle.length());
                for (size_t i=0; i<tp.posture.size(); i++) {
                    tp.posture[i] = co.angle[i];
                }
                for (size_t i=0; i<co.lines.length(); i++) {
                    hrp::Vector3 p0, p1;
                    p0[0] = co.lines[i][0][0];
                    p0[1] = co.lines[i][0][1];
                    p0[2] = co.lines[i][0][2];
                    p1[0] = co.lines[i][1][0];
                    p1[1] = co.lines[i][1][1];
                    p1[2] = co.lines[i][1][2];
                    tp.lines.push_back(std::make_pair(p0,p1));
                }
#endif
                log.add(co);
           } catch (...) {
               std::cerr << "exceptoin in stateUpdate" << std::endl;
           }
        }

        // update Scene
    }

    try {
	orb->destroy();
    }
    catch(...){

    }

    return 0;
}

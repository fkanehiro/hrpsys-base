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
#include "hrpsys/util/ProjectUtil.h"
#include "hrpsys/util/GLbody.h"
#include "hrpsys/util/GLlink.h"
#include "hrpsys/util/GLutil.h"
#include "hrpsys/util/SDLUtil.h"
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
        std::cerr << "Usage:" << argv[0] << " project.xml [-rh RobotHardwareComponent] [-sh StateHolder component] [-size size] [-bg r g b] [-host localhost] [-port 2809] [-interval 100] [-nogui]" << std::endl;
        return 1;
    }

    Project prj;
    if (!prj.parse(argv[1])){
        std::cerr << "failed to parse " << argv[1] << std::endl;
        return 1;
    }

    char *rhname = NULL, *shname = NULL, *hostname = NULL;
    int wsize = 0, port=0, interval=0;
    float bgColor[] = {0,0,0};
    bool orbinitref = false, gui = true;
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
        }else if(strcmp(argv[i], "-host")==0){
            hostname = argv[++i];
        }else if(strcmp(argv[i], "-port")==0){
            port = atoi(argv[++i]);
        }else if(strcmp(argv[i], "-interval")==0){
            interval = atoi(argv[++i]);
        }else if(strcmp(argv[i], "-nogui")==0){
            gui = false;
        }else if(strcmp(argv[i], "-ORBInitRef")==0){
            orbinitref = true;
        }
    }
    char **nargv = argv;
    int nargc = argc;
    if(orbinitref == false){
        nargv = (char **)malloc((argc+2)*sizeof(char *));
        for(int i = 0; i < nargc; i++) nargv[i] = argv[i];
        char buf1[] = "-ORBInitRef";
        char buf2[256];
        sprintf(buf2, "NameService=corbaloc:iiop:%s:%d/NameService", hostname==NULL?"localhost":hostname, port==0?2809:port);
        nargv[nargc++] = buf1;
        nargv[nargc++] = buf2;
    }

    //================= CORBA =========================
    for(int i = 0; i < nargc-1; i++) {
        if(strcmp(nargv[i], "-ORBInitRef")==0) {
            std::cerr << "[monitor] Starting CORBA Client with " << nargv[i+1] << std::endl;
        }
    }
    CORBA::ORB_var orb;
    CosNaming::NamingContext_var namingContext;
    
    try {
        orb = CORBA::ORB_init(nargc, nargv);
        
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
        std::cerr << "[monitor] Failed to initialize CORBA " << std::endl << ex._rep_id() << std::endl;
        return -1;
    }catch (const std::string& error){
        std::cerr << "[monitor] Failed to initialize CORBA " << std::endl << error << std::endl;
        return -1;
    }

    //================= logger ======================
    LogManager<TimedRobotState> log; 
    log.enableRingBuffer(5000);
    
    //================= monitor ======================
    RobotHardwareClientView rhview = prj.RobotHardwareClient();
    if ( hostname != NULL ) rhview.hostname = hostname;
    if ( port != 0 ) rhview.port = port;
    if ( interval != 0 ) rhview.interval = interval;
    std::cerr << "[monitor] Monitor service with " << rhview.hostname << "@" << rhview.port << ", " << 1000/rhview.interval << "Hz" << std::endl;
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
    if ( gui ) {
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
    }
    else
    {
        std::vector<hrp::ColdetLinkPairPtr> pairs;
        ModelLoader_var modelloader = getModelLoader(namingContext);
        if (CORBA::is_nil(modelloader)){
            std::cerr << "openhrp-model-loader is not running" << std::endl;
            return 1;
        }
        BodyFactory factory = boost::bind(createBody, _1, _2, modelloader);

        // add bodies
        hrp::BodyPtr body;
        for (std::map<std::string, ModelItem>::iterator it=prj.models().begin();
             it != prj.models().end(); it++){
            const std::string name
                = it->second.rtcName == "" ? it->first : it->second.rtcName;
            hrp::BodyPtr tmp_body = factory(name, it->second);
            if (tmp_body){
                tmp_body->setName(name);
                if(tmp_body->numJoints() > 0)
                    body = tmp_body;
                else
                    std::cerr << "[monitor] Skipping non-robot model (" << name << ")" << std::endl;
            }
        }
        if (!body) {
            std::cerr << "[monitor]  Monitoring " << body->name() << std::endl;
            return -1;
        }
        monitor.start();
        while(monitor.oneStep()){
            log.updateIndex();
            monitor.showStatus(body);
        }
        monitor.stop();
    }
    try {
        orb->destroy();
    }
    catch(...){
        
    }
    
    return 0;
}

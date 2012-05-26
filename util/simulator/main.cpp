#include <fstream>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <rtm/Manager.h>
#include <rtm/CorbaNaming.h>
#include <hrpModel/ModelLoaderUtil.h>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include <SDL_thread.h>
#include "util/GLbody.h"
#include "util/GLlink.h"
#include "util/GLutil.h"
#include "util/Project.h"
#include "util/OpenRTMUtil.h"
#include "util/SDLUtil.h"
#include "util/BodyRTC.h"
#include "Simulator.h"
#include "GLscene.h"

using namespace std;
using namespace hrp;
using namespace OpenHRP;

hrp::BodyPtr createBody(const std::string& name, const ModelItem& mitem,
                        ModelLoader_ptr modelloader, bool usebbox)
{
    std::cout << "createBody(" << name << "," << mitem.url << ")" << std::endl;
    RTC::Manager& manager = RTC::Manager::instance();
    std::string args = "BodyRTC?instance_name="+name;
    BodyRTCPtr body = (BodyRTC *)manager.createComponent(args.c_str());
    BodyInfo_var binfo;
    if (usebbox){
        ModelLoader::ModelLoadOption mlopt;
        mlopt.readImage = false;
        mlopt.AABBtype = OpenHRP::ModelLoader::AABB_NUM;
        mlopt.AABBdata.length(mitem.joint.size());
        std::map<std::string, JointItem>::const_iterator it;
        int i=0;
        for (it = mitem.joint.begin(); it != mitem.joint.end(); it++){
            mlopt.AABBdata[i++] = 1;
        }
        binfo = modelloader->getBodyInfoEx(mitem.url.c_str(), mlopt);
    }else{
        binfo = modelloader->getBodyInfo(mitem.url.c_str());
    }
    if (!loadBodyFromBodyInfo(body, binfo, true)){
        std::cerr << "failed to load model[" << mitem.url << "]" << std::endl;
        manager.deleteComponent(body.get());
        return hrp::BodyPtr();
    }else{
        body->createDataPorts();
        return body;
    }
}

int main(int argc, char* argv[]) 
{
    bool display = true, realtime=false, usebbox=false, endless=false;
    for (int i=0; i<argc; i++){
        if (strcmp("-nodisplay",argv[i])==0){
            display = false;
        }else if(strcmp("-realtime", argv[i])==0){
            realtime = true;
        }else if(strcmp("-usebbox", argv[i])==0){
            usebbox = true;
        }else if(strcmp("-endless", argv[i])==0){
            endless = true;
        }
    }

    Project prj;
    if (!prj.parse(argv[1])){
        std::cerr << "failed to parse " << argv[1] << std::endl;
        return 1;
    }

    //================= OpenRTM =========================
    RTC::Manager* manager;
    int rtmargc=0;
    std::vector<char *> rtmargv;
    for (int i=1; i<argc; i++){
        if (strcmp(argv[i], "-nodisplay") 
            && strcmp(argv[i], "-realtime")
            && strcmp(argv[i], "-usebbox")
            && strcmp(argv[i], "-endless")
            ){
            rtmargv.push_back(argv[i]);
            rtmargc++;
        }
    }
    manager = RTC::Manager::init(rtmargc, rtmargv.data());
    manager->init(rtmargc, rtmargv.data());
    BodyRTC::moduleInit(manager);
    manager->activateManager();
    manager->runManager(true);

    std::string nameServer = manager->getConfig()["corba.nameservers"];
    int comPos = nameServer.find(",");
    if (comPos < 0){
        comPos = nameServer.length();
    }
    nameServer = nameServer.substr(0, comPos);
    RTC::CorbaNaming naming(manager->getORB(), nameServer.c_str());

    ModelLoader_var modelloader = getModelLoader(CosNaming::NamingContext::_duplicate(naming.getRootContext()));
    //==================== Viewer setup ===============
    LogManager<SceneState> log;
    GLscene scene(&log);
    Simulator simulator(&log);

    SDLwindow window(&scene, &log, &simulator);
    if (display){
        window.init();
        for (std::map<std::string, ModelItem>::iterator it=prj.models().begin();
             it != prj.models().end(); it++){
            OpenHRP::ModelLoader::ModelLoadOption opt;
            opt.readImage = true;
            opt.AABBdata.length(0);
            opt.AABBtype = OpenHRP::ModelLoader::AABB_NUM;
            OpenHRP::BodyInfo_var binfo
                = modelloader->loadBodyInfoEx(it->second.url.c_str(), opt);
            GLbody *glbody = new GLbody();
            hrp::BodyPtr body(glbody);
            hrp::loadBodyFromBodyInfo(body, binfo, false, GLlinkFactory);
            loadShapeFromBodyInfo(glbody, binfo);
            body->setName(it->first);
            scene.WorldBase::addBody(body);
        }
    }

    //================= setup Simulator ======================
    BodyFactory factory = boost::bind(createBody, _1, _2, modelloader,usebbox);
    simulator.init(prj, factory);
    simulator.realTime(realtime);
    if (endless){
        simulator.setTotalTime(0);
        log.enableRingBuffer(50000);
    }

    std::cout << "timestep = " << prj.timeStep() << ", total time = " 
              << prj.totalTime() << std::endl;

    if (display){
        simulator.start();
        while(window.oneStep());
        simulator.stop();
    }else{
        while (simulator.oneStep());
    }

    manager->shutdown();

    return 0;
}

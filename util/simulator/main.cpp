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
#include "util/GLbodyRTC.h"
#include "util/GLlink.h"
#include "util/GLutil.h"
#include "util/Project.h"
#include "util/OpenRTMUtil.h"
#include "util/SDLUtil.h"
#include "util/BVutil.h"
#include "Simulator.h"
#include "GLscene.h"

using namespace std;
using namespace hrp;
using namespace OpenHRP;

hrp::BodyPtr createBody(const std::string& name, const ModelItem& mitem,
                        ModelLoader_ptr modelloader, GLscene *scene,
                        bool usebbox)
{
    std::cout << "createBody(" << name << "," << mitem.url << ")" << std::endl;
    RTC::Manager& manager = RTC::Manager::instance();
    std::string args = "GLbodyRTC?instance_name="+name;
    GLbodyRTC *glbodyrtc = (GLbodyRTC *)manager.createComponent(args.c_str());
    hrp::BodyPtr body = hrp::BodyPtr(glbodyrtc);
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
    if (!loadBodyFromBodyInfo(body, binfo, true, GLlinkFactory)){
        std::cerr << "failed to load model[" << mitem.url << "]" << std::endl;
        manager.deleteComponent(glbodyrtc);
        return hrp::BodyPtr();
    }else{
        for (std::map<std::string, JointItem>::const_iterator it2=mitem.joint.begin();
             it2 != mitem.joint.end(); it2++){
            hrp::Link *link = body->link(it2->first);
            if (!link) continue;
            if (link) link->isHighGainMode = it2->second.isHighGain;
            if (it2->second.collisionShape == ""){
                // do nothing
            }else if (it2->second.collisionShape == "convex hull"){
                convertToConvexHull(link);
            }else if (it2->second.collisionShape == "AABB"){
                convertToAABB(link);
            }else{
                std::cerr << "unknown value of collisionShape property:" 
                          << it2->second.collisionShape << std::endl;
            }
        }
        if (usebbox) convertToAABB(body);
        for (size_t i=0; i<mitem.inports.size(); i++){
            glbodyrtc->createInPort(mitem.inports[i]);
        }
        for (size_t i=0; i<mitem.outports.size(); i++){
            glbodyrtc->createOutPort(mitem.outports[i]);
        }
        loadShapeFromBodyInfo(glbodyrtc, binfo);
        body->setName(name);
        scene->addBody(body);
        return body;
    }
}

void print_usage(char* progname)
{
    std::cerr << "Usage:" << progname << " [project file] [options]" << std::endl;
    std::cerr << "Options:" << std::endl;
    std::cerr << " -nodisplay         : headless mode" << std::endl;
    std::cerr << " -realtime          : syncronize to real world time" << std::endl;
    std::cerr << " -usebbox           : use bounding box for collision detection" << std::endl;
    std::cerr << " -endless           : endless mode" << std::endl;
    std::cerr << " -showsensors       : visualize sensors" << std::endl;
    std::cerr << " -size [pixels]     : specify window size in pixels" << std::endl;
    std::cerr << " -no-default-lights : disable ambient light (simulation environment will be dark)" << std::endl;
    std::cerr << " -max-edge-length [value] : specify maximum length of polygon edge (if exceed, polygon will be divided to improve rendering quality)" << std::endl;
    std::cerr << " -max-log-length [value] : specify maximum size of the log" << std::endl;
    std::cerr << " -exit-on-finish    : exit the program when the simulation finish" << std::endl;
    std::cerr << " -record            : record the simulation as movie" << std::endl;
    std::cerr << " -bg [r] [g] [b]    : specify background color" << std::endl;
    std::cerr << " -h --help          : show this help message" << std::endl;
}

int main(int argc, char* argv[]) 
{
    bool display = true, usebbox=false;
    bool showsensors = false;
    int wsize = 0;
    bool useDefaultLights = true;
    double maxEdgeLen = 0;
    bool exitOnFinish = false;
    bool record = false;
    double maxLogLen = 60;
    bool realtime = false;
    bool endless = false;

    if (argc <= 1){
        print_usage(argv[0]);
        return 1;
    }

    float bgColor[]={0,0,0};
    for (int i=1; i<argc; i++){
        if (strcmp("-nodisplay",argv[i])==0){
            display = false;
        }else if(strcmp("-realtime", argv[i])==0){
            realtime = true;
        }else if(strcmp("-usebbox", argv[i])==0){
            usebbox = true;
        }else if(strcmp("-endless", argv[i])==0){
            endless = true;
        }else if(strcmp("-showsensors", argv[i])==0){
            showsensors = true;
        }else if(strcmp("-size", argv[i])==0){
            wsize = atoi(argv[++i]);
        }else if(strcmp("-no-default-lights", argv[i])==0){
            useDefaultLights = false;
        }else if(strcmp("-max-edge-length", argv[i])==0){
            maxEdgeLen = atof(argv[++i]);
        }else if(strcmp("-max-log-length", argv[i])==0){
            maxLogLen = atof(argv[++i]);
        }else if(strcmp("-exit-on-finish", argv[i])==0){
            exitOnFinish = true;
        }else if(strcmp("-record", argv[i])==0){
            record = true;
            exitOnFinish = true;
        }else if(strcmp("-bg", argv[i])==0){
            bgColor[0] = atof(argv[++i]);
            bgColor[1] = atof(argv[++i]);
            bgColor[2] = atof(argv[++i]);
        }else if(strcmp("-h", argv[i])==0 || strcmp("--help", argv[i])==0){
            print_usage(argv[0]);
            return 1;
        }
    }

    Project prj;
    if (!prj.parse(argv[1])){
        std::cerr << "failed to parse " << argv[1] << std::endl;
        return 1;
    }
    if (realtime){
        prj.realTime(true);
    }
    if (endless){
        prj.totalTime(0);
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
            && strcmp(argv[i], "-showsensors")
            && strcmp(argv[i], "-size")
            && strcmp(argv[i], "-no-default-lights")
            && strcmp(argv[i], "-max-edge-length")
            && strcmp(argv[i], "-max-log-length")
            && strcmp(argv[i], "-exit-on-finish")
            && strcmp(argv[i], "-record")
            && strcmp(argv[i], "-bg")
            ){
            rtmargv.push_back(argv[i]);
            rtmargc++;
        }
    }
    manager = RTC::Manager::init(rtmargc, rtmargv.data());
    manager->init(rtmargc, rtmargv.data());
    GLbodyRTC::moduleInit(manager);
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
    if (CORBA::is_nil(modelloader)){
        std::cerr << "openhrp-model-loader is not running" << std::endl;
        return 1;
    }
    //==================== Viewer setup ===============
    LogManager<SceneState> log;
    GLscene scene(&log);
    scene.setBackGroundColor(bgColor);
    scene.showSensors(showsensors);
    scene.maxEdgeLen(maxEdgeLen);
    scene.showCollision(prj.view().showCollision);
    Simulator simulator(&log);

    SDLwindow window(&scene, &log, &simulator);
    if (display){
        window.init(wsize, wsize);
        if (!useDefaultLights) scene.defaultLights(false);
        window.setView(prj.view().T);
        scene.showFloorGrid(prj.view().showScale);
    }

    //================= setup Simulator ======================
    BodyFactory factory = boost::bind(createBody, _1, _2, modelloader, &scene, usebbox);
    simulator.init(prj, factory);
    if (!prj.totalTime()){
        log.enableRingBuffer(maxLogLen/prj.timeStep());
    }

    std::cout << "timestep = " << prj.timeStep() << ", total time = " 
              << prj.totalTime() << std::endl;

    if (display){
        simulator.start();
        while(window.oneStep()){
            if (exitOnFinish && !simulator.isRunning()) break;
        };
        simulator.stop();
        if (record){
            log.record(10);
            while(window.oneStep()){
                if (!log.isRecording()) break;
            }
        }
    }else{
        while (simulator.oneStep());
    }

    manager->shutdown();

    return 0;
}

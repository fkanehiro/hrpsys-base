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
#include "Simulator.h"
#include "SDLUtil.h"
#include "GLmodel.h"
#include "Project.h"
#include "OpenRTMUtil.h"
#include "BodyRTC.h"

using namespace std;
using namespace hrp;
using namespace OpenHRP;

hrp::BodyPtr createBody(const std::string& name, const std::string& url,
                        RTC::CorbaNaming *naming)
{
    std::cout << "createBody(" << name << "," << url << ")" << std::endl;
    RTC::Manager& manager = RTC::Manager::instance();
    std::string args = "BodyRTC?instance_name="+name;
    BodyRTCPtr body = (BodyRTC *)manager.createComponent(args.c_str());
    if (!loadBodyFromModelLoader(body, url.c_str(), 
                                 CosNaming::NamingContext::_duplicate(naming->getRootContext()),
                                 true)){
        std::cerr << "failed to load model[" << url << "]" << std::endl;
        manager.deleteComponent(body.get());
        return hrp::BodyPtr();
    }else{
        body->createDataPorts();
        return body;
    }
}

int threadMain(void *arg)
{
    Simulator *simulator = (Simulator *)arg;
    while(simulator->oneStep());
}

int main(int argc, char* argv[]) 
{
    bool display = true;
    for (int i=0; i<argc; i++){
        if (strcmp("-nodisplay",argv[i])==0){
            display = false;
        }
    }

    Project prj;
    if (!prj.parse(argv[1])){
        std::cerr << "failed to parse " << argv[1] << std::endl;
        return 1;
    }

    //================= OpenRTM =========================
    RTC::Manager* manager;
    manager = RTC::Manager::init(argc, argv);
    manager->init(argc, argv);
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

    //==================== Viewer setup ===============
    GLscene *scene = NULL;
    if (display){
        glutInit(&argc, argv);
        scene = GLscene::getInstance();
    }
    SDLwindow window(scene);
    if (display){
        window.init();
        scene->init();

        for (std::map<std::string, ModelItem>::iterator it=prj.models().begin();
             it != prj.models().end(); it++){
            OpenHRP::BodyInfo_var binfo
                = loadBodyInfo(it->second.url.c_str(), 
                               CosNaming::NamingContext::_duplicate(naming.getRootContext()));
            GLbody *body = new GLbody(binfo);
            scene->addBody(it->first, body);
        }
    }

    //================= setup World ======================
    BodyFactory factory = boost::bind(createBody, _1, _2, &naming);
    Simulator simulator;
    simulator.init(prj, factory, scene);

    std::cout << "timestep = " << prj.timeStep() << ", total time = " 
              << prj.totalTime() << std::endl;

    if (display){
        SDL_Thread *thread;
        thread = SDL_CreateThread(threadMain, (void *)&simulator);
        while(1) {
            //std::cerr << "t = " << world.currentTime() << std::endl;
            if (!window.processEvents()) break;
            window.draw();
            window.swapBuffers();
        }
        simulator.stopSimulation();
        SDL_WaitThread(thread, NULL);
    }else{
        threadMain(&simulator);
    }

    manager->shutdown();

    return 0;
}

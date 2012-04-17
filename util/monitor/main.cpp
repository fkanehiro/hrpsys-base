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
#include "SDLUtil.h"
#include "GLmodel.h"
#include "util/Project.h"
#include "OpenRTMUtil.h"

using namespace std;
using namespace hrp;
using namespace OpenHRP;

int threadMain(void *arg)
{

}

int main(int argc, char* argv[]) 
{
    Project prj;
    if (!prj.parse(argv[1])){
        std::cerr << "failed to parse " << argv[1] << std::endl;
        return 1;
    }

    //================= OpenRTM =========================
    RTC::Manager* manager;
    manager = RTC::Manager::init(argc, argv);
    manager->init(argc, argv);
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
    GLscene *scene = NULL;
    glutInit(&argc, argv); // for bitmap fonts
    scene = GLscene::getInstance();

    SDLwindow window(scene);
    window.init();
    scene->init();
    
    for (std::map<std::string, ModelItem>::iterator it=prj.models().begin();
         it != prj.models().end(); it++){
        OpenHRP::BodyInfo_var binfo
            = modelloader->loadBodyInfo(it->second.url.c_str());
        GLbody *body = new GLbody(binfo);
        scene->addBody(it->first, body);
    }

    //================= setup World ======================
    std::cout << "timestep = " << prj.timeStep() << ", total time = " 
              << prj.totalTime() << std::endl;

    SDL_Thread *thread;
    thread = SDL_CreateThread(threadMain, NULL);
    while(1) {
        //std::cerr << "t = " << world.currentTime() << std::endl;
        if (!window.processEvents()) break;
        window.draw();
        window.swapBuffers();
    }
    SDL_WaitThread(thread, NULL);

    manager->shutdown();

    return 0;
}

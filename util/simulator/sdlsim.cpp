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
#include "util/Project.h"
#include "util/OpenRTMUtil.h"
#include "Simulator.h"
#include "SDLUtil.h"
#include "GLmodel.h"
#include "BodyRTC.h"

using namespace std;
using namespace hrp;
using namespace OpenHRP;

hrp::BodyPtr createBody(const std::string& name, const ModelItem& mitem,
                        ModelLoader_ptr modelloader)
{
    std::cout << "createBody(" << name << "," << mitem.url << ")" << std::endl;
    RTC::Manager& manager = RTC::Manager::instance();
    std::string args = "BodyRTC?instance_name="+name;
    BodyRTCPtr body = (BodyRTC *)manager.createComponent(args.c_str());
    ModelLoader::ModelLoadOption mlopt;
    mlopt.readImage = false;
    mlopt.AABBtype = OpenHRP::ModelLoader::AABB_NUM;
    mlopt.AABBdata.length(mitem.joint.size());
     std::map<std::string, JointItem>::const_iterator it;
    int i=0;
    for (it = mitem.joint.begin(); it != mitem.joint.end(); it++){
        //mlopt.AABBdata[i++] = it->second.NumOfAABB;
        mlopt.AABBdata[i++] = 1;
    }
    //BodyInfo_var binfo = modelloader->getBodyInfoEx(mitem.url.c_str(), mlopt);
    BodyInfo_var binfo = modelloader->getBodyInfo(mitem.url.c_str());
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

    ModelLoader_var modelloader = getModelLoader(CosNaming::NamingContext::_duplicate(naming.getRootContext()));
    //==================== Viewer setup ===============
    GLscene *scene = NULL;
    Simulator simulator;

    if (display){
        glutInit(&argc, argv);
        scene = GLscene::getInstance();
    }
    SDLwindow window(scene, &simulator);
    if (display){
        window.init();
        scene->init();

        for (std::map<std::string, ModelItem>::iterator it=prj.models().begin();
             it != prj.models().end(); it++){
            OpenHRP::BodyInfo_var binfo
                = modelloader->loadBodyInfo(it->second.url.c_str());
            GLbody *body = new GLbody(binfo);
            scene->addBody(it->first, body);
        }
    }

    //================= setup Simulator ======================
    BodyFactory factory = boost::bind(createBody, _1, _2, modelloader);
    simulator.init(prj, factory, scene);

    std::cout << "timestep = " << prj.timeStep() << ", total time = " 
              << prj.totalTime() << std::endl;

    if (display){
        simulator.start();
        while(1) {
            //std::cerr << "t = " << world.currentTime() << std::endl;
            if (!window.processEvents()) break;
            window.draw();
            window.swapBuffers();
        }
        simulator.stop();
    }else{
        while (simulator.oneStep());
    }

    manager->shutdown();

    return 0;
}

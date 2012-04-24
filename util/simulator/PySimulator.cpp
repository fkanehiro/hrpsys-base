#include <fstream>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/python.hpp>

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
#include "util/SDLUtil.h"
#include "Simulator.h"
#include "GLscene.h"
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

class PySimulator
{
public:
    PySimulator() 
        : scene(&log), window(&scene, &log, &simulator){
        int argc = 1;
        char *argv[] = {(char *)"dummy"};
        manager = RTC::Manager::init(argc, argv);
        manager->init(argc, argv);
        BodyRTC::moduleInit(manager);
        manager->activateManager();
        manager->runManager(true);
        window.start();
    }
    ~PySimulator(){
        window.stop();
        manager->shutdown();
    }
    void init(std::string fname){
        Project prj;
        if (!prj.parse(fname)){
            std::cerr << "failed to parse " << fname << std::endl;
            return;
        }
        
        RTC::Manager* manager = &RTC::Manager::instance();
        std::string nameServer = manager->getConfig()["corba.nameservers"];
        int comPos = nameServer.find(",");
        if (comPos < 0){
            comPos = nameServer.length();
        }
        nameServer = nameServer.substr(0, comPos);
        RTC::CorbaNaming naming(manager->getORB(), nameServer.c_str());
        
        ModelLoader_var modelloader = getModelLoader(CosNaming::NamingContext::_duplicate(naming.getRootContext()));
        //==================== Viewer setup ===============
        for (std::map<std::string, ModelItem>::iterator it=prj.models().begin();
             it != prj.models().end(); it++){
            OpenHRP::BodyInfo_var binfo
                = modelloader->loadBodyInfo(it->second.url.c_str());
            scene.addBody(it->first, binfo);
        }
        //================= setup Simulator ======================
        BodyFactory factory = boost::bind(createBody, _1, _2, modelloader);
        simulator.init(prj, factory, &log);
        
        std::cout << "timestep = " << prj.timeStep() << ", total time = " 
                  << prj.totalTime() << std::endl;
    }
    void simulate(double time){
        simulator.totalTime(simulator.currentTime()+time);
        while(simulator.oneStep());
    }
    void start(double time){
        simulator.totalTime(simulator.currentTime()+time);
        simulator.start();
    }
    void stop(){
        simulator.stop();
    }
    void realTime(bool flag){
        simulator.realTime(flag);
    } 
private:  
    LogManager<SceneState> log;
    GLscene scene;
    Simulator simulator;
    SDLwindow window;
    RTC::Manager* manager;
};

BOOST_PYTHON_MODULE( simulator )
{
    boost::python::class_<PySimulator>("Simulator")
        .def("init", &PySimulator::init)
        .def("simulate", &PySimulator::simulate)
        .def("realTime", &PySimulator::realTime)
        .def("start", &PySimulator::start)
        .def("stop", &PySimulator::stop)
        ;
}

#include <fstream>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/python.hpp>
#include <boost/foreach.hpp>
#include <boost/range/value_type.hpp>
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
#include "PyBody.h"
#include "PyLink.h"

using namespace std;
using namespace hrp;
using namespace OpenHRP;

static hrp::Link *createLink() { return new PyLink(); }
hrp::BodyPtr createBody(const std::string& name, const ModelItem& mitem,
                        ModelLoader_ptr modelloader)
{
    std::cout << "createBody(" << name << "," << mitem.url << ")" << std::endl;
    RTC::Manager& manager = RTC::Manager::instance();
    std::string args = "PyBody?instance_name="+name;
    PyBody *pybody = (PyBody *)manager.createComponent(args.c_str());
    hrp::BodyPtr body = hrp::BodyPtr(pybody);
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
        manager.deleteComponent(pybody);
        return hrp::BodyPtr();
    }else{
        pybody->createDataPorts();
        return body;
    }
}

class PySimulator
{
public:
    PySimulator() 
        : scene(&log), simulator(&log), window(&scene, &log, &simulator){
        int argc = 1;
        char *argv[] = {(char *)"dummy"};
        manager = RTC::Manager::init(argc, argv);
        manager->init(argc, argv);
        PyBody::moduleInit(manager);
        manager->activateManager();
        manager->runManager(true);
        window.start();
    }
    ~PySimulator(){
        window.stop();
        manager->shutdown();
    }
    PyBody* loadBody(std::string name, std::string url){
        RTC::Manager* manager = &RTC::Manager::instance();
        std::string nameServer = manager->getConfig()["corba.nameservers"];
        int comPos = nameServer.find(",");
        if (comPos < 0){
            comPos = nameServer.length();
        }
        nameServer = nameServer.substr(0, comPos);
        RTC::CorbaNaming naming(manager->getORB(), nameServer.c_str());
        
        ModelLoader_var modelloader = getModelLoader(CosNaming::NamingContext::_duplicate(naming.getRootContext()));
        OpenHRP::BodyInfo_var binfo
            = modelloader->loadBodyInfo(url.c_str());
        scene.addBody(name, binfo);
        std::string args = "PyBody?instance_name="+name;
        PyBody *pybody = (PyBody *)manager->createComponent(args.c_str());
        hrp::BodyPtr body = hrp::BodyPtr(pybody);
        if (!loadBodyFromBodyInfo(body, binfo, true, createLink)){
            std::cerr << "failed to load model[" << url << "]" << std::endl;
            manager->deleteComponent(pybody);
            return NULL;
        }else{
            pybody->createDataPorts();
            body->setName(name);
            simulator.addBody(body);
            return pybody;
        }
    }
    void loadProject(std::string fname){
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
        simulator.init(prj, factory);
        
        std::cout << "timestep = " << prj.timeStep() << ", total time = " 
                  << prj.totalTime() << std::endl;
    }
    void simulate(){
        while(simulator.oneStep());
    }
    void simulate(double time){
        simulator.totalTime(simulator.currentTime()+time);
        simulate();
    }
    void start(double time){
        simulator.totalTime(simulator.currentTime()+time);
        simulator.start();
    }
    void stop(){
        simulator.stop();
    }
    void wait(){
        simulator.wait();
    }
    void realTime(bool flag){
        simulator.realTime(flag);
    } 
    void endless(bool flag){
        if (flag){
            simulator.totalTime(0);
            log.enableRingBuffer(50000);
        }
    } 
    void setTimeStep(double t){
        simulator.timeStep(t);
    }
    double getTimeStep(){
        return simulator.timeStep();
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
    using namespace boost::python;

    class_<PySimulator>("Simulator")
        .def("loadBody", &PySimulator::loadBody, return_value_policy<manage_new_object>())
        .def("loadProject", &PySimulator::loadProject)
        .def("simulate", (void(PySimulator::*)())&PySimulator::simulate)
        .def("simulate", (void(PySimulator::*)(double))&PySimulator::simulate)
        .def("realTime", &PySimulator::realTime)
        .def("endless", &PySimulator::endless)
        .def("start", &PySimulator::start)
        .def("stop", &PySimulator::stop)
        .def("wait", &PySimulator::wait)
        .add_property("timeStep", 
                      &PySimulator::getTimeStep, &PySimulator::setTimeStep)
        ;

    class_<PyBody>("Body", no_init)
        .def("calcForwardKinematics", &PyBody::calcForwardKinematics)
        .def("rootLink", &PyBody::rootLink, return_internal_reference<>())
        .def("link", &PyBody::link, return_internal_reference<>())
        .def("links", &PyBody::links)
        .def("joint", &PyBody::joint, return_internal_reference<>())
        .def("joints", &PyBody::joints)
        .add_property("name", &PyBody::getName, &PyBody::setName)
        .add_property("p", &PyBody::getPosition, &PyBody::setPosition)
        .add_property("R", &PyBody::getOrientation, &PyBody::setOrientation)
        .add_property("q", &PyBody::getPosture, &PyBody::setPosture)
        ;

    class_<PyLink>("Link", no_init)
        .add_property("p", &PyLink::getPosition, &PyLink::setPosition)
        .add_property("R", &PyLink::getOrientation, &PyLink::setOrientation)
        .add_property("q", &PyLink::getPosture, &PyLink::setPosture)
        ;
}

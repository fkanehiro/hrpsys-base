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
#include "PyBody.h"
#include "PyLink.h"
#include "PySimulator.h"

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

PySimulator::PySimulator() 
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

PySimulator::~PySimulator(){
    window.stop();
    manager->shutdown();
}

PyBody* PySimulator::loadBody(std::string name, std::string url){
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
    pybody->setListener(this);
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

void PySimulator::loadProject(std::string fname){
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
    BodyFactory factory = boost::bind(::createBody, _1, _2, modelloader);
    simulator.init(prj, factory);
    
    std::cout << "timestep = " << prj.timeStep() << ", total time = " 
              << prj.totalTime() << std::endl;
}

void PySimulator::simulate()
{
    while(simulator.oneStep());
}

void PySimulator::simulate(double time)
{
    simulator.totalTime(simulator.currentTime()+time);
    simulate();
}

void PySimulator::start(double time)
{
    simulator.totalTime(simulator.currentTime()+time);
    simulator.start();
}

void PySimulator::stop()
{
    simulator.stop();
}

void PySimulator::wait()
{
    simulator.wait();
}

void PySimulator::realTime(bool flag)
{
    simulator.realTime(flag);
} 

void PySimulator::endless(bool flag)
{
    if (flag){
        simulator.totalTime(0);
        log.enableRingBuffer(50000);
    }
} 

void PySimulator::setTimeStep(double t)
{
    simulator.timeStep(t);
}

double PySimulator::getTimeStep()
{
    return simulator.timeStep();
}

void PySimulator::clear()
{
    simulator.clear();
    scene.clear();
}

void PySimulator::play()
{
    log.play();
}

void PySimulator::pause()
{
    log.play();
}

void PySimulator::notifyChanged()
{
    simulator.appendLog();
}

PyBody *PySimulator::createBody(std::string name)
{
    RTC::Manager* manager = &RTC::Manager::instance();
    std::string args = "PyBody?instance_name="+name;
    PyBody *pybody = (PyBody *)manager->createComponent(args.c_str());
    pybody->setListener(this);
    pybody->setName(name);
    PyLink *root = new PyLink();
    pybody->setRootLink(root);
    
    hrp::BodyPtr body = hrp::BodyPtr(pybody);
    simulator.addBody(body);

    return pybody;
}

PyObject *PySimulator::bodies()
{
    boost::python::list retval;
    for (int i=0; i<simulator.numBodies(); i++){
        PyBody *b = (PyBody *)simulator.body(i);
        //retval.append(boost::python::object(b));
        retval.append(b);
    }
    return boost::python::incref(retval.ptr());
}

void PySimulator::addCollisionCheckPair(PyBody *b1, PyBody *b2)
{
    simulator.addCollisionCheckPair(b1, b2);
}

bool PySimulator::oneStep()
{
    return simulator.oneStep();
}

double PySimulator::time()
{
    return simulator.currentTime();
}

void PySimulator::initialize()
{
    simulator.initialize();
}

BOOST_PYTHON_MODULE( simulator )
{
    using namespace boost::python;

    class_<PySimulator>("Simulator")
        .def("loadBody", &PySimulator::loadBody, return_internal_reference<>())
        .def("createBody", &PySimulator::createBody, return_internal_reference<>())
        .def("addCollisionCheckPair", &PySimulator::addCollisionCheckPair)
        .def("loadProject", &PySimulator::loadProject)
        .def("oneStep", &PySimulator::oneStep)
        .def("simulate", (void(PySimulator::*)())&PySimulator::simulate)
        .def("simulate", (void(PySimulator::*)(double))&PySimulator::simulate)
        .def("realTime", &PySimulator::realTime)
        .def("endless", &PySimulator::endless)
        .def("start", &PySimulator::start)
        .def("stop", &PySimulator::stop)
        .def("wait", &PySimulator::wait)
        .def("clear", &PySimulator::clear)
        .def("play", &PySimulator::play)
        .def("pause", &PySimulator::pause)
        .def("bodies", &PySimulator::bodies)
        .def("initialize", &PySimulator::initialize)
        .add_property("timeStep", 
                      &PySimulator::getTimeStep, &PySimulator::setTimeStep)
        .add_property("time", &PySimulator::time)
        ;

    class_<PyBody>("Body", no_init)
        .def("calcForwardKinematics", &PyBody::calcForwardKinematics)
        .def("rootLink", &PyBody::rootLink, return_internal_reference<>())
        .def("link", &PyBody::link, return_internal_reference<>())
        .def("links", &PyBody::links)
        .def("joint", &PyBody::joint, return_internal_reference<>())
        .def("joints", &PyBody::joints)
        .def("calcTotalMass", &PyBody::calcTotalMass)
        .def("calcCM", &PyBody::calcCM)
        .def("totalMass", &PyBody::totalMass)
        .def("numJoints", &PyBody::numJoints)
        .def("numLinks", &PyBody::numLinks)
        .add_property("name", &PyBody::getName, &PyBody::setName)
        .add_property("p", &PyBody::getPosition, &PyBody::setPosition)
        .add_property("R", &PyBody::getRotation, &PyBody::setRotation)
        .add_property("q", &PyBody::getPosture, &PyBody::setPosture)
        ;

    class_<PyLink>("Link", no_init)
        .def("createLink", &PyLink::createLink, return_internal_reference<>()) 
        .def("parent", &PyLink::getParent, return_internal_reference<>())
        .def("children", &PyLink::getChildren)
        .def_readwrite("name", &PyLink::name)
        .def_readwrite("jointId", &PyLink::jointId)
        .def_readwrite("m", &PyLink::m)
        .def_readwrite("u", &PyLink::u)
        .add_property("q", &PyLink::getPosture, &PyLink::setPosture)
        .add_property("p", &PyLink::getPosition, &PyLink::setPosition)
        .add_property("R", &PyLink::getRotation, &PyLink::setRotation)
        .add_property("c", &PyLink::getCoM, &PyLink::setCoM)
        .add_property("I", &PyLink::getInertia, &PyLink::setInertia)
        .add_property("v", &PyLink::getLinVel, &PyLink::setLinVel)
        .add_property("w", &PyLink::getAngVel, &PyLink::setAngVel)
        .add_property("jointType", &PyLink::getJointType, &PyLink::setJointType)
        ;
}

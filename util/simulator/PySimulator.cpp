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
#include "util/GLlink.h"
#include "util/GLutil.h"
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
                        ModelLoader_ptr modelloader, GLscene *scene)
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
    if (!loadBodyFromBodyInfo(body, binfo, true, GLlinkFactory)){
        std::cerr << "failed to load model[" << mitem.url << "]" << std::endl;
        manager.deleteComponent(pybody);
        return hrp::BodyPtr();
    }else{
        pybody->createDataPorts();
        loadShapeFromBodyInfo(pybody, binfo);
        scene->addBody(body);
        return body;
    }
}

PySimulator::PySimulator() : 
    manager(NULL), Simulator(&log), scene(&log), window(&scene, &log, this)
{
}

PySimulator::~PySimulator(){
    window.stop();
    if (manager) manager->shutdown();
}

void PySimulator::initViewer()
{
    window.start();
}

void PySimulator::initRTCmanager()
{
    int argc=1;
    char *argv[] = {(char *)"dummy"};
    initRTCmanager(argc, argv);
}

void PySimulator::initRTCmanager(int argc, char **argv)
{
    manager = RTC::Manager::init(argc, argv);
    manager->init(argc, argv);
    PyBody::moduleInit(manager);
    manager->activateManager();
    manager->runManager(true);
}

void PySimulator::initRTCmanager(PyObject *pyo)
{
    std::vector<char *> args(PySequence_Size(pyo)+1);
    args[0] = (char *)"dummy";
    for (int i=0; i<PySequence_Size(pyo); i++) {
        args[i+1] = boost::python::extract<char *>(PySequence_GetItem(pyo, i));
    }
    initRTCmanager(args.size(), &args[0]);
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
        addBody(body);
        loadShapeFromBodyInfo(pybody, binfo);
        scene.addBody(body);
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
    //================= setup Simulator ======================
    BodyFactory factory = boost::bind(::createBody, _1, _2, modelloader, 
                                      &scene);
    init(prj, factory);
    
    std::cout << "timestep = " << prj.timeStep() << ", total time = " 
              << prj.totalTime() << std::endl;
}

void PySimulator::simulate()
{
    while(oneStep());
}

void PySimulator::simulate(double time)
{
    totalTime(currentTime()+time);
    simulate();
}

void PySimulator::start(double time)
{
    totalTime(currentTime()+time);
    Simulator::start();
}

void PySimulator::realTime(bool flag)
{
    realTime(flag);
} 

void PySimulator::endless(bool flag)
{
    if (flag){
        totalTime(0);
        log.enableRingBuffer(50000);
    }
} 

void PySimulator::clear()
{
    clear();
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
    appendLog();
}

PyBody *PySimulator::createBody(std::string name)
{
    RTC::Manager* manager = &RTC::Manager::instance();
    std::string args = "PyBody?instance_name="+name;
    PyBody *pybody = (PyBody *)manager->createComponent(args.c_str());
    pybody->setListener(this);
    pybody->setName(name);
    PyLink *root = new PyLink();
    root->name = "root";
    pybody->setRootLink(root);
    
    hrp::BodyPtr body = hrp::BodyPtr(pybody);
    addBody(body);
    scene.addBody(body);

    return pybody;
}

PyObject *PySimulator::bodies()
{
    boost::python::list retval;
    for (int i=0; i<numBodies(); i++){
        PyBody *b = dynamic_cast<PyBody *>(body(i).get());
        //retval.append(boost::python::object(b));
        retval.append(b);
    }
    return boost::python::incref(retval.ptr());
}

void PySimulator::addCollisionCheckPair(PyBody *b1, PyBody *b2)
{
    Simulator::addCollisionCheckPair(b1, b2);
}


BOOST_PYTHON_MODULE( simulator )
{
    using namespace boost::python;

    class_<PySimulator>("Simulator")
        .def("initRTCmanager", (void (PySimulator::*)())&PySimulator::initRTCmanager)
        .def("initRTCmanager", (void (PySimulator::*)(PyObject *))&PySimulator::initRTCmanager)
        .def("initViewer", &PySimulator::initViewer)
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
                      &PySimulator::timeStep, &PySimulator::setTimeStep)
        .add_property("time", &PySimulator::currentTime)
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
        .def("addChildLink", &PyLink::addChildLink, return_internal_reference<>()) 
        .def("addShapeFromFile", &PyLink::addShapeFromFile)
        .def("addCube", &PyLink::addCube)
        .def("parent", &PyLink::getParent, return_internal_reference<>())
        .def("children", &PyLink::getChildren)
        .def("showAxes", &PyLink::showAxes)
        .def_readwrite("name", &PyLink::name)
        .def_readwrite("jointId", &PyLink::jointId)
        .def_readwrite("m", &PyLink::m)
        .def_readwrite("u", &PyLink::u)
        .def_readwrite("Ir", &PyLink::Ir)
        .def_readwrite("gearRatio", &PyLink::gearRatio)
        .def_readwrite("dq", &PyLink::dq)
        .def_readwrite("ddq", &PyLink::ddq)
        .def_readwrite("isHighGainMode", &PyLink::isHighGainMode)
        .add_property("q", &PyLink::getPosture, &PyLink::setPosture)
        .add_property("p", &PyLink::getPosition, &PyLink::setPosition)
        .add_property("R", &PyLink::getRotation, &PyLink::setRotation)
        .add_property("b", &PyLink::getRelPosition, &PyLink::setRelPosition)
        .add_property("Rs", &PyLink::getRelRotation, &PyLink::setRelRotation)
        .add_property("a", &PyLink::getRotationAxis, &PyLink::setRotationAxis)
        .add_property("d", &PyLink::getTranslationAxis, &PyLink::setTranslationAxis)
        .add_property("c", &PyLink::getCoM, &PyLink::setCoM)
        .add_property("I", &PyLink::getInertia, &PyLink::setInertia)
        .add_property("v", &PyLink::getLinVel, &PyLink::setLinVel)
        .add_property("w", &PyLink::getAngVel, &PyLink::setAngVel)
        .add_property("jointType", &PyLink::getJointType, &PyLink::setJointType)
        ;
}

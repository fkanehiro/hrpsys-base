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
#include "hrpsys/util/GLbody.h"
#include "hrpsys/util/GLlink.h"
#include "hrpsys/util/GLutil.h"
#include "hrpsys/util/Project.h"
#include "hrpsys/util/OpenRTMUtil.h"
#include "hrpsys/util/BVutil.h"
#include "PyBody.h"
#include "PyLink.h"
#include "PyShape.h"
#include "PySimulator.h"

using namespace std;
using namespace hrp;
using namespace OpenHRP;

static hrp::Link *createLink() { return new PyLink(); }
hrp::BodyPtr createBody(const std::string& name, const ModelItem& mitem,
                        ModelLoader_ptr modelloader, GLscene *scene,
                        bool usebbox)
{
    std::cout << "createBody(" << name << "," << mitem.url << ")" << std::endl;
    RTC::Manager& manager = RTC::Manager::instance();
    std::string args = "PyBody?instance_name="+name;
    PyBody *pybody = (PyBody *)manager.createComponent(args.c_str());
    hrp::BodyPtr body = hrp::BodyPtr(pybody);
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
        manager.deleteComponent(pybody);
        return hrp::BodyPtr();
    }else{
        if (usebbox) convertToAABB(body);
        for (std::map<std::string, JointItem>::const_iterator it2=mitem.joint.begin();
             it2 != mitem.joint.end(); it2++){
            hrp::Link *link = body->link(it2->first);
            if (!link) continue;
            link->isHighGainMode = it2->second.isHighGain;
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
        for (size_t i=0; i<mitem.inports.size(); i++){
            pybody->createInPort(mitem.inports[i]);
        }
        for (size_t i=0; i<mitem.outports.size(); i++){
            pybody->createOutPort(mitem.outports[i]);
        }
        loadShapeFromBodyInfo(pybody, binfo, createPyShape);
        body->setName(name);
        scene->addBody(body);
        return body;
    }
}

PySimulator::PySimulator() : 
    Simulator(&log), scene(&log), window(&scene, &log, this), manager(NULL),
    useBBox(false), maxLogLen(60)
{
    initRTCmanager();
}

PySimulator::PySimulator(PyObject *pyo) : 
    Simulator(&log), scene(&log), window(&scene, &log, this), manager(NULL),
    useBBox(false)
{
    initRTCmanager(pyo);
}

PySimulator::~PySimulator(){
    stop();
    window.stop();
    clear();
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
    OpenHRP::BodyInfo_var binfo;
    try{
        binfo = modelloader->loadBodyInfo(url.c_str());
    }catch(OpenHRP::ModelLoader::ModelLoaderException ex){
        std::cerr << ex.description << std::endl;
        return NULL;
    }
    std::string args = "PyBody?instance_name="+name;
    PyBody *pybody = (PyBody *)manager->createComponent(args.c_str());
    pybody->setListener(this);
    hrp::BodyPtr body = hrp::BodyPtr(pybody);
    if (!loadBodyFromBodyInfo(body, binfo, true, createLink)){
        std::cerr << "failed to load model[" << url << "]" << std::endl;
        manager->deleteComponent(pybody);
        return NULL;
    }else{
        if (useBBox) convertToAABB(body);
        body->setName(name);
        addBody(body);
        loadShapeFromBodyInfo(pybody, binfo, createPyShape);
        scene.addBody(body);
        return pybody;
    }
}

bool PySimulator::loadProject(std::string fname){
    clear();

    Project prj;
    if (!prj.parse(fname)){
        std::cerr << "failed to parse " << fname << std::endl;
        return false;
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
                                      &scene, useBBox);
    init(prj, factory);
    for (unsigned int i=0; i<numBodies(); i++){
        PyBody *pybody = dynamic_cast<PyBody *>(body(i).get());
        pybody->setListener(this);
    }
    window.setView(prj.view().T);
    
    std::cout << "timestep = " << prj.timeStep() << ", total time = " 
              << prj.totalTime() << std::endl;
    return true;
}

void PySimulator::simulate()
{
    while(oneStep());
}

void PySimulator::simulate(double time)
{
    setTotalTime(currentTime()+time);
    simulate();
}

void PySimulator::start(double time)
{
    setTotalTime(currentTime()+time);
    Simulator::start();
}

void PySimulator::endless(bool flag)
{
    if (flag){
        setTotalTime(0);
        log.enableRingBuffer(maxLogLen/timeStep());
    }
} 

void PySimulator::clear()
{
    Simulator::clear();
    log.clear();
    if (window.isRunning()) scene.requestClear();
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
    for (unsigned int i=0; i<numBodies(); i++){
        PyBody *b = dynamic_cast<PyBody *>(body(i).get());
        retval.append(boost::python::ptr(b));
    }
    return boost::python::incref(retval.ptr());
}

void PySimulator::addCollisionCheckPair(PyBody *b1, PyBody *b2)
{
    Simulator::addCollisionCheckPair(b1, b2);
}

void PySimulator::capture(std::string fname)
{
    scene.requestCapture(fname.c_str());
}

unsigned int PySimulator::logLength()
{
    return log.length();
}

PyBody *PySimulator::getBody(std::string name)
{
    return dynamic_cast<PyBody *>(body(name).get());
}

bool PySimulator::showSensors()
{
    return scene.showSensors();
}

void PySimulator::setShowSensors(bool flag)
{
    scene.showSensors(flag);
}

void PySimulator::setMaxLogLength(double len)
{
    maxLogLen = len;
}

double PySimulator::maxLogLength()
{
    return maxLogLen;
}

void PySimulator::reset()
{
    log.clear();
    setCurrentTime(0.0);
    for (unsigned int i=0; i<numBodies(); i++){
        body(i)->initializeConfiguration();
    }
    checkCollision();
    appendLog();
}

void PySimulator::setUseBBox(bool flag)
{
    useBBox = flag;
}

void PySimulator::setWindowSize(int s)
{
    window.setSize(s,s);
}

BOOST_PYTHON_MODULE( hrpsysext )
{
    using namespace boost::python;

    class_<PySimulator, boost::noncopyable>("Simulator")
        .def(init<PyObject *>())
        .def("initViewer", &PySimulator::initViewer)
        .def("loadBody", &PySimulator::loadBody, return_internal_reference<>())
        .def("createBody", &PySimulator::createBody, return_internal_reference<>())
        .def("addCollisionCheckPair", &PySimulator::addCollisionCheckPair)
        .def("loadProject", &PySimulator::loadProject)
        .def("oneStep", &PySimulator::oneStep)
        .def("simulate", (void(PySimulator::*)())&PySimulator::simulate)
        .def("simulate", (void(PySimulator::*)(double))&PySimulator::simulate)
        .def("realTime", &PySimulator::realTime)
        .def("useBBox", &PySimulator::setUseBBox)
        .def("windowSize", &PySimulator::setWindowSize)
        .def("endless", &PySimulator::endless)
        .def("start", &PySimulator::start)
        .def("stop", &PySimulator::stop)
        .def("wait", &PySimulator::wait)
        .def("clear", &PySimulator::clear)
        .def("reset", &PySimulator::reset)
        .def("play", &PySimulator::play)
        .def("pause", &PySimulator::pause)
        .def("capture", &PySimulator::capture)
        .def("logLength", &PySimulator::logLength)
        .def("body", &PySimulator::getBody, return_internal_reference<>())
        .def("bodies", &PySimulator::bodies)
        .def("initialize", &PySimulator::initialize)
        .add_property("timeStep", 
                      &PySimulator::timeStep, &PySimulator::setTimeStep)
        .add_property("time", &PySimulator::currentTime)
        .add_property("totalTime", 
                      &PySimulator::totalTime, &PySimulator::setTotalTime)
        .add_property("showSensors", 
                      &PySimulator::showSensors, &PySimulator::setShowSensors)
        .add_property("maxLogLength", 
                      &PySimulator::maxLogLength, &PySimulator::setMaxLogLength)
        ;

    class_<PyBody, boost::noncopyable>("Body", no_init)
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
        .def("creteInPort", &PyBody::createInPort)
        .def("creteOutPort", &PyBody::createOutPort)
        .add_property("name", &PyBody::getName, &PyBody::setName)
        .add_property("p", &PyBody::getPosition, &PyBody::setPosition)
        .add_property("R", &PyBody::getRotation, &PyBody::setRotation)
        .add_property("q", &PyBody::getPosture, &PyBody::setPosture)
        ;

    class_<PyLink, boost::noncopyable>("Link", no_init)
        .def("addChildLink", &PyLink::addChildLink, return_internal_reference<>()) 
        .def("addShapeFromFile", &PyLink::addShapeFromFile)
        .def("addCube", &PyLink::addCube, return_internal_reference<>())
        .def("parent", &PyLink::getParent, return_internal_reference<>())
        .def("children", &PyLink::getChildren)
        .def("showAxes", &PyLink::showAxes)
        .def("shapes", &PyLink::shapes)
        .def("highlight", &PyLink::highlight)
        .def_readwrite("name", &PyLink::name)
        .def_readwrite("m", &PyLink::m)
        .def_readwrite("u", &PyLink::u)
        .def_readwrite("Ir", &PyLink::Ir)
        .def_readwrite("gearRatio", &PyLink::gearRatio)
        .def_readwrite("dq", &PyLink::dq)
        .def_readwrite("ddq", &PyLink::ddq)
        .def_readwrite("isHighGainMode", &PyLink::isHighGainMode)
        .add_property("jointId", &PyLink::getJointId, &PyLink::setJointId)
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

    class_<PyShape, boost::noncopyable>("Shape", no_init)
        .add_property("b", &PyShape::getRelPosition, &PyShape::setRelPosition)
        .add_property("Rs", &PyShape::getRelRotation, &PyShape::setRelRotation)
        .add_property("diffuse", &PyShape::getDiffuseColor, &PyShape::setDiffuseColor)
        ;
}

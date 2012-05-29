#include <iostream>
#include <rtm/Manager.h>
#include <rtm/CorbaNaming.h>
#include <hrpModel/ModelLoaderUtil.h>
#include <util/GLutil.h>
#include "PyLink.h"
#include "PyBody.h"
#include "PyShape.h"
#include "PyUtil.h"

PyLink::PyLink()
{
}

PyLink::~PyLink()
{
    //std::cout << "~PyLink()" << std::endl;
}

PyObject *PyLink::getPosition()
{
    boost::python::list retval;
    VectorToPyList(p, retval);
    return boost::python::incref(retval.ptr());
}

void PyLink::setPosition(PyObject *v)
{
    if (PySequence_Size(v) != 3) return;
    PyListToVector(v, p);
    notifyChanged();
}

PyObject *PyLink::getRotation()
{
    boost::python::list retval;
    const hrp::Matrix33 R = attitude();
    Matrix33ToPyList(R, retval);
    return boost::python::incref(retval.ptr());
}

void PyLink::setRotation(PyObject *v)
{
    hrp::Matrix33 R;
    int n = PySequence_Size(v);
    if (n == 9){
        PyListToMatrix33(v, R);
    }else if (n == 4){
        hrp::Vector3 axis;
        for (int i=0; i<3; i++) {
            axis(i) = boost::python::extract<double>(PySequence_GetItem(v, i));
        }
        double angle = boost::python::extract<double>(PySequence_GetItem(v, 3));
        hrp::calcRodrigues(R, axis, angle);
    }else if (n == 3){
        hrp::Vector3 rpy;
        PyListToVector(v, rpy);
        hrp::calcRotFromRpy(R, rpy[0], rpy[1], rpy[2]);
    }else{
        return;
    }
    setAttitude(R);
    notifyChanged();
}

PyObject *PyLink::getRelPosition()
{
    boost::python::list retval;
    VectorToPyList(b, retval);
    return boost::python::incref(retval.ptr());
}

void PyLink::setRelPosition(PyObject *v)
{
    if (PySequence_Size(v) != 3) return;
    if (parent){
        PyListToVector(v, b);
        GLcoordinates::setPosition(b);
    }else{
        PyListToVector(v, p);
        GLcoordinates::setPosition(p);
    }
    notifyChanged();
}

PyObject *PyLink::getRelRotation()
{
    boost::python::list retval;
    Matrix33ToPyList(Rs, retval);
    return boost::python::incref(retval.ptr());
}

void PyLink::setRelRotation(PyObject *v)
{
    int n = PySequence_Size(v);
    if (n == 9){
        PyListToMatrix33(v, Rs);
    }else if (n == 4){
        hrp::Vector3 axis;
        for (int i=0; i<3; i++) {
            axis(i) = boost::python::extract<double>(PySequence_GetItem(v, i));
        }
        double angle = boost::python::extract<double>(PySequence_GetItem(v, 3));
        hrp::calcRodrigues(Rs, axis, angle);
    }else if (n == 3){
        hrp::Vector3 rpy;
        PyListToVector(v, rpy);
        hrp::calcRotFromRpy(Rs, rpy[0], rpy[1], rpy[2]);
    }else{
        return;
    }
    GLcoordinates::setRotation(Rs);
    notifyChanged();
}

double PyLink::getPosture()
{
    return q;
}

void PyLink::setPosture(double v)
{
    q = v;
    setQ(q);
    notifyChanged();
}

PyObject *PyLink::getCoM()
{
    boost::python::list retval;
    VectorToPyList(c, retval);
    return boost::python::incref(retval.ptr());
}

void PyLink::setCoM(PyObject *v)
{
    if (PySequence_Size(v) != 3) return;
    PyListToVector(v, c);
}

PyObject *PyLink::getRotationAxis()
{
    boost::python::list retval;
    VectorToPyList(a, retval);
    notifyChanged();
    return boost::python::incref(retval.ptr());
}

void PyLink::setRotationAxis(PyObject *v)
{
    if (PySequence_Size(v) != 3) return;
    PyListToVector(v, a);
}

PyObject *PyLink::getTranslationAxis()
{
    boost::python::list retval;
    VectorToPyList(d, retval);
    return boost::python::incref(retval.ptr());
}

void PyLink::setTranslationAxis(PyObject *v)
{
    if (PySequence_Size(v) != 3) return;
    PyListToVector(v, d);
}

PyObject *PyLink::getInertia()
{
    boost::python::list retval;
    Matrix33ToPyList(I, retval);
    return boost::python::incref(retval.ptr());
}

void PyLink::setInertia(PyObject *v)
{
    if (PySequence_Size(v) != 9) return;
    PyListToMatrix33(v, I);
}

void PyLink::notifyChanged()
{
    PyBody *pybody = dynamic_cast<PyBody *>(body);
    pybody->notifyChanged(PyBody::KINEMATICS);
}

PyObject *PyLink::getLinVel()
{
    boost::python::list retval;
    VectorToPyList(v, retval);
    return boost::python::incref(retval.ptr());
}

void PyLink::setLinVel(PyObject *pyo)
{
    if (PySequence_Size(pyo) != 3) return;
    PyListToVector(pyo, v);
}

PyObject *PyLink::getAngVel()
{
    boost::python::list retval;
    VectorToPyList(w, retval);
    return boost::python::incref(retval.ptr());
}

void PyLink::setAngVel(PyObject *v)
{
    if (PySequence_Size(v) != 3) return;
    PyListToVector(v, w);
}

PyLink* PyLink::addChildLink(std::string name)
{
    PyLink *l = new PyLink();
    l->name = name;
    addChild(l);
    PyBody *pybody = dynamic_cast<PyBody *>(body);
    pybody->notifyChanged(PyBody::STRUCTURE);
    return l;
}

void PyLink::addShapeFromFile(std::string url)
{
    RTC::Manager* manager = &RTC::Manager::instance();
    std::string nameServer = manager->getConfig()["corba.nameservers"];
    int comPos = nameServer.find(",");
    if (comPos < 0){
        comPos = nameServer.length();
    }
    nameServer = nameServer.substr(0, comPos);
    RTC::CorbaNaming naming(manager->getORB(), nameServer.c_str());
    
    OpenHRP::ModelLoader_var modelloader = hrp::getModelLoader(CosNaming::NamingContext::_duplicate(naming.getRootContext()));
    OpenHRP::ModelLoader::ModelLoadOption opt;
    opt.readImage = true;
    opt.AABBdata.length(0);
    opt.AABBtype = OpenHRP::ModelLoader::AABB_NUM;
    OpenHRP::BodyInfo_var binfo = modelloader->getBodyInfoEx(url.c_str(), opt);
    OpenHRP::LinkInfoSequence_var lis = binfo->links();
    loadShapeFromLinkInfo(this, lis[0], binfo, createPyShape);
}

PyShape *PyLink::addCube(double x, double y, double z)
{
    PyShape *shape = new PyShape();
    loadCube(shape, x,y,z);
    addShape(shape);
    return shape;
}

PyLink *PyLink::getParent()
{
    return (PyLink *)parent;
}

PyObject *PyLink::getChildren()
{
    boost::python::list retval;
    hrp::Link *l = child;
    while(l){
        retval.append(boost::python::object((PyLink *)l));
        l = l->sibling;
    }
    return boost::python::incref(retval.ptr());
}

void PyLink::setJointType(std::string type)
{
    if(type == "fixed" ){
        jointType = Link::FIXED_JOINT;
    } else if(type == "free" ){
        jointType = Link::FREE_JOINT;
    } else if(type == "rotate" ){
        jointType = Link::ROTATIONAL_JOINT;
    } else if(type == "slide" ){
        jointType = Link::SLIDE_JOINT;
    } else {
        jointType = Link::FREE_JOINT;
    }
}

std::string PyLink::getJointType()
{
    switch(jointType){
    case Link::FIXED_JOINT:
        return "fixed";
    case Link::FREE_JOINT:
        return "free";
    case Link::ROTATIONAL_JOINT:
        return "rotate";
    case Link::SLIDE_JOINT:
        return "slide";
    default:
        return "";
    }
}

PyObject *PyLink::shapes()
{
    boost::python::list retval;
    for (size_t i=0; i<m_shapes.size(); i++){
        retval.append(boost::python::ptr((PyShape *)m_shapes[i]));
    }
    return boost::python::incref(retval.ptr());
}

int PyLink::getJointId()
{
    return jointId; 
}

void PyLink::setJointId(int id)
{
    jointId = id;
    PyBody *pybody = dynamic_cast<PyBody *>(body);
    pybody->notifyChanged(PyBody::STRUCTURE);
}

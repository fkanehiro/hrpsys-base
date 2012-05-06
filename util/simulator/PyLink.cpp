#include <iostream>
#include "PyLink.h"
#include "PyBody.h"

PyLink::~PyLink()
{
    //std::cout << "~PyLink()" << std::endl;
}

template<class T>
void VectorToPyList(const T& v, boost::python::list &l)
{
    for (int i=0; i<v.size(); i++){
        l.append(boost::python::object(v[i]));
    }
}

static void Matrix33ToPyList(const hrp::Matrix33& M, boost::python::list &l)
{
    for (int i=0; i<3; i++){
        for (int j=0; j<3; j++){
            l.append(boost::python::object(M(i,j)));
        }
    }
}

template<class T>
void PyListToVector(PyObject *pyo, T& v)
{
    for (int i=0; i<PySequence_Size(pyo); i++) {
        v[i] = boost::python::extract<double>(PySequence_GetItem(pyo, i));
    }
}

static void PyListToMatrix33(PyObject *pyo, hrp::Matrix33& M)
{
    for (int i=0; i<9; i++) {
        M(i/3, i%3) = boost::python::extract<double>(PySequence_GetItem(pyo, i));
    }
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

double PyLink::getPosture()
{
    return q;
}

void PyLink::setPosture(double v)
{
    q = v;
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
    PyBody *pybody = (PyBody *)body;
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

PyLink* PyLink::createLink()
{
    PyLink *l = new PyLink();
    addChild(l);
    PyBody *pybody = (PyBody *)body;
    pybody->notifyChanged(PyBody::STRUCTURE);
    return l;
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

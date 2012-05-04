#ifndef __PYBODY_H__
#define __PYBODY_H__

#include <boost/python.hpp>
#include <boost/foreach.hpp>
#include <boost/range/value_type.hpp>
#include <hrpModel/Link.h>
#include "PyLink.h"
#include "PyBody.h"

const char* PyBody::pybody_spec[] =
{
    "implementation_id", "PyBody",
    "type_name",         "PyBody",
    "description",       "PyBody component",
    "version",           "0.1",
    "vendor",            "AIST",
    "category",          "Generic",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables

    ""
};

PyBody::PyBody(RTC::Manager* manager) : BodyRTC(manager)
{
}

PyBody::~PyBody() 
{
}

void PyBody::setPosition(PyObject *v)
{
    rootLink()->setPosition(v);
}

void PyBody::setOrientation(PyObject *v)
{
    rootLink()->setOrientation(v);
}

void PyBody::setPosture(PyObject *v)
{
    if (PySequence_Size(v) != numJoints()) return;
    for (int i=0; i<numJoints(); i++) {
        hrp::Link *j = joint(i);
        if (j) j->q = boost::python::extract<double>(PySequence_GetItem(v, i));
    }
}

PyObject *PyBody::getPosition()
{
    return rootLink()->getPosition();
}

PyObject *PyBody::getOrientation()
{
    return rootLink()->getOrientation();
}

PyObject *PyBody::getPosture()
{
    boost::python::list retval;
    for (int i=0; i<numJoints(); i++){
        hrp::Link *j = joint(i);
        double q = j ? j->q : 0;
        retval.append(boost::python::object(q));
    }
    return boost::python::incref(retval.ptr());
}

void PyBody::calcForwardKinematics()
{
    Body::calcForwardKinematics();
}

PyLink *PyBody::rootLink()
{
    return (PyLink *)Body::rootLink();
}

PyLink *PyBody::link(std::string name)
{
    return (PyLink *)Body::link(name);
}

PyObject *PyBody::links()
{
    boost::python::list retval;
    for (int i=0; i<numLinks(); i++){
        PyLink *l = (PyLink *)Body::link(i);
        retval.append(boost::python::object(l));
    }
    return boost::python::incref(retval.ptr());
}

PyLink *PyBody::joint(int i)
{
    return (PyLink *)Body::joint(i);
}
PyObject *PyBody::joints()
{
    boost::python::list retval;
    for (int i=0; i<numJoints(); i++){
        PyLink *l = (PyLink *)Body::joint(i);
        retval.append(boost::python::object(l));
    }
    return boost::python::incref(retval.ptr());
}

std::string PyBody::getName()
{
    return Body::name();
}

void PyBody::setName(std::string name)
{
    Body::setName(name);
}

template <class _Delete>
void DummyDelete(RTC::RTObject_impl* rtc)
{
}

void PyBody::moduleInit(RTC::Manager* manager)
{
    coil::Properties profile(pybody_spec);
    manager->registerFactory(profile,
                             RTC::Create<PyBody>,
                             DummyDelete<PyBody>
                             //RTC::Delete<PyBody>
        );
}

#endif

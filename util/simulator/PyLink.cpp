#include "PyLink.h"

void PyLink::setPosition(PyObject *v)
{
    if (PySequence_Size(v) != 3) return;
    for (int i=0; i<3; i++) {
        p[i] = boost::python::extract<double>(PySequence_GetItem(v, i));
    }
}

void PyLink::setOrientation(PyObject *v)
{
    if (PySequence_Size(v) != 9) return;
    hrp::Matrix33 R;
    for (int i=0; i<9; i++) {
        R(i/3, i%3) = boost::python::extract<double>(PySequence_GetItem(v, i));
    }
    setAttitude(R);
}

void PyLink::setPosture(double v)
{
    q = v;
}

PyObject *PyLink::getPosition()
{
    boost::python::list retval;
    for (int i=0; i<3; i++){
        retval.append(boost::python::object(p[i]));
    }
    return boost::python::incref(retval.ptr());
}

PyObject *PyLink::getOrientation()
{
    boost::python::list retval;
    const hrp::Matrix33 R = attitude();
    for (int i=0; i<3; i++){
        for (int j=0; j<3; j++){
            retval.append(boost::python::object(R(i,j)));
        }
    }
    return boost::python::incref(retval.ptr());
}

double PyLink::getPosture()
{
    return q;
}

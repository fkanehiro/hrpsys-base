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
    hrp::Matrix33 R;
    int n = PySequence_Size(v);
    if (n == 9){
        for (int i=0; i<9; i++) {
            R(i/3, i%3) = boost::python::extract<double>(PySequence_GetItem(v, i));
        }
    }else if (n == 4){
        hrp::Vector3 axis;
        for (int i=0; i<3; i++) {
            axis(i) = boost::python::extract<double>(PySequence_GetItem(v, i));
        }
        double angle = boost::python::extract<double>(PySequence_GetItem(v, 3));
        hrp::calcRodrigues(R, axis, angle);
    }else if (n == 3){
        double rpy[3];
        for (int i=0; i<3; i++) {
            rpy[i] = boost::python::extract<double>(PySequence_GetItem(v, i));
        }
        hrp::calcRotFromRpy(R, rpy[0], rpy[1], rpy[2]);
    }else{
        return;
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

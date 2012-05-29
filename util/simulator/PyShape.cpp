#include "PyShape.h"
#include "PyUtil.h"

PyObject *PyShape::getRelPosition()
{
    boost::python::list retval;
    VectorToPyList(getPosition(), retval);
    return boost::python::incref(retval.ptr());
}

void PyShape::setRelPosition(PyObject *v)
{
    if (PySequence_Size(v) != 3) return;
    hrp::Vector3 b;
    PyListToVector(v, b);
    GLcoordinates::setPosition(b);
}

PyObject *PyShape::getRelRotation()
{
    boost::python::list retval;
    hrp::Matrix33 Rs = getRotation();
    Matrix33ToPyList(Rs, retval);
    return boost::python::incref(retval.ptr());
}

void PyShape::setRelRotation(PyObject *v)
{
    int n = PySequence_Size(v);
    hrp::Matrix33 Rs;
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
}

PyObject *PyShape::getDiffuseColor()
{
    boost::python::list retval;
    for (int i=0; i<4; i++) retval.append(m_diffuse[i]);
    return boost::python::incref(retval.ptr());
}

void PyShape::setDiffuseColor(PyObject *v)
{
    if (PySequence_Size(v) != 4) return;

    for (int i=0; i<PySequence_Size(v); i++) {
        m_diffuse[i] = boost::python::extract<double>(PySequence_GetItem(v, i));
    }
    compile();
}

GLshape *createPyShape()
{
    return new PyShape();
}
